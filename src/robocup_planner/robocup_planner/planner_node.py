"""
RoboCup Planner Node

Subscribes to the task topic, computes the full plan, then runs the
reactive executor in a background thread while the ROS2 node spins
normally in the main thread.

Interfaces:
  Sub  /sml/task              sml_msgs/Task         — task definition
  Sub  <wb_ready_topic>       std_msgs/Int32         — workbench product_id ready
  Act  navigate_to_station    sml_msgs/NavTask       — navigate to station
  Act  wb_task                sml_msgs/WbTask        — workbench work
  Srv  /amr_robot_command     sml_msgs/ArmCommand    — arm pick/place

Blocking helper methods (navigate, arm_*, wb_task) are called from the
executor thread and use threading.Event to wait for ROS2 async results.
"""

import threading
from collections import Counter
from typing import Optional

import rclpy
from rclpy.action import ActionClient
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSProfile, QoSReliabilityPolicy

TASK_QOS = QoSProfile(
    depth=1,
    durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
    reliability=QoSReliabilityPolicy.RELIABLE,
)

from sml_msgs.action import NavTask, WbTask
from sml_msgs.msg import Task
from sml_msgs.srv import ArmCommand
from std_msgs.msg import Int32
from std_srvs.srv import Trigger

from robocup_planner.planning.aidlist_builder import compute_net_aidlist
from robocup_planner.planning.cargo_allocator import CargoAllocator
from robocup_planner.planning.distance_calculator import DistanceCalculator
from robocup_planner.planning.midlist_builder import (
    build_full_midlist,
    build_mid,
    build_bidlist,
    build_storage_midlist,
    check_storage_satisfies,
    merge_into_midlist,
)
from robocup_planner.execution.executor import Executor, Plan
from robocup_planner.product_catalog import (
    is_intransit_eligible,
    BATCH_TO_MATERIAL,
    BATCH_COUNT,
)

# Workbench WbTask goal strings
WB_PRODUCE = 'PRODUCE'
WB_RECYCLE = 'RECYCLE'

# Arm ArmCommand.srv action strings (matches amr_robot_node / mock_arm_node)
ARM_PICK = 'LOAD'
ARM_PLACE = 'UNLOAD'
ARM_DELIVER = 'UNLOAD'


class PlannerNode(Node):

    def __init__(self):
        super().__init__('robocup_planner')

        # --- Parameters ---
        try:
            from ament_index_python.packages import get_package_share_directory
            import os as _os
            _default_wp = _os.path.join(
                get_package_share_directory('robocup_planner'),
                'config',
                'robocup_waypoint.yaml',
            )
        except Exception:
            _default_wp = ''
        self.declare_parameter('waypoint_yaml', _default_wp)
        self.declare_parameter('task_topic', '/sml/task')
        self.declare_parameter('nav_action', 'navigate_to_station')
        self.declare_parameter('wb_action', 'wb_task')
        self.declare_parameter('arm_service', '/amr_robot_command')
        self.declare_parameter('wb_ready_topic', '/workbench/product_ready')
        self.declare_parameter('post_process_service', '/robocup_navigator/post_process')
        self.declare_parameter('driving_velocity', 0.5)
        self.declare_parameter('parking_duration', 1.5)
        self.declare_parameter('exiting_duration', 1.0)

        wp_path = self.get_parameter('waypoint_yaml').get_parameter_value().string_value
        task_topic = self.get_parameter('task_topic').get_parameter_value().string_value
        nav_action = self.get_parameter('nav_action').get_parameter_value().string_value
        wb_action = self.get_parameter('wb_action').get_parameter_value().string_value
        arm_service = self.get_parameter('arm_service').get_parameter_value().string_value
        wb_ready_topic = self.get_parameter('wb_ready_topic').get_parameter_value().string_value
        post_process_service = self.get_parameter('post_process_service').get_parameter_value().string_value

        if not wp_path:
            self.get_logger().warning("waypoint_yaml parameter is empty; distances will be inf")
            self._calc: Optional[DistanceCalculator] = None
        else:
            self._calc = DistanceCalculator(wp_path)

        # --- ROS interfaces ---
        self._task_sub = self.create_subscription(
            Task, task_topic, self._on_task, TASK_QOS
        )
        self._wb_ready_sub = self.create_subscription(
            Int32, wb_ready_topic, self._on_wb_ready, 10
        )
        self._nav_client = ActionClient(self, NavTask, nav_action)
        self._wb_client = ActionClient(self, WbTask, wb_action)
        self._arm_client = self.create_client(ArmCommand, arm_service)
        self._post_process_client = self.create_client(Trigger, post_process_service)

        # Active executor (one at a time)
        self._executor_thread: Optional[threading.Thread] = None
        self._active_executor: Optional[Executor] = None
        self._exec_lock = threading.Lock()

        # Pending in-transit ASSEMBLE events (arm works asynchronously during travel)
        self._intransit_events: list = []
        self._intransit_lock = threading.Lock()

        self.get_logger().info("RoboCup Planner ready — waiting for task")

    # ------------------------------------------------------------------
    # Task callback — triggers planning + execution
    # ------------------------------------------------------------------

    def _on_task(self, msg: Task) -> None:
        with self._exec_lock:
            if self._executor_thread and self._executor_thread.is_alive():
                self.get_logger().warning(
                    "New task received while execution is running — ignoring"
                )
                return

        self.get_logger().info("Task received — planning...")
        try:
            plan = self._plan(msg)
        except Exception as e:
            self.get_logger().error(f"Planning failed: {e}")
            return

        executor = Executor(plan, self)
        self._active_executor = executor

        thread = threading.Thread(target=executor.run, daemon=True, name='executor')
        self._executor_thread = thread
        thread.start()

    # ------------------------------------------------------------------
    # Workbench ready signal — sets the executor's event
    # ------------------------------------------------------------------

    def _on_wb_ready(self, msg: Int32) -> None:
        self.get_logger().info(
            f"Workbench signal: product {msg.data} ready"
        )
        if self._active_executor is not None:
            self._active_executor.wb_signal.set()

    # ------------------------------------------------------------------
    # Planning phase
    # ------------------------------------------------------------------

    def _plan(self, msg: Task) -> Plan:
        from sml_msgs.msg import Station as StationMsg

        # Parse orders
        produce_ids = [o.product_id for o in msg.order_list if o.order_type == 1]
        recycle_ids = [o.product_id for o in msg.order_list if o.order_type == 2]

        self.get_logger().info(
            f"Plan: produce={produce_ids}, recycle={recycle_ids}"
        )

        # Categorise stations — separate regular storage from batch stations
        storage_stations = []    # material_ids 1-8
        batch_stations_1080 = [] # batch_ids 10-80 (known type, 5 blocks each)
        batch_stations_90 = []   # mix batch ID 90 (unknown content)
        workbench_station_id = None
        customer_station_id = None
        customer_stations = []

        for st in msg.arena_layout:
            if st.station_type in (StationMsg.ST_STORAGE, StationMsg.ST_HYBRID):
                mids = [int(m) for m in st.material_ids]
                regular = [m for m in mids if 1 <= m <= 8]
                b1080 = [m for m in mids if 10 <= m <= 80]
                b90 = [m for m in mids if m == 90]
                if regular:
                    storage_stations.append({
                        'station_id': st.station_id,
                        'material_ids': regular,
                    })
                if b1080:
                    batch_stations_1080.append({
                        'station_id': st.station_id,
                        'batch_ids': b1080,
                    })
                if b90:
                    batch_stations_90.append({
                        'station_id': st.station_id,
                        'batch_ids': [90],
                    })
                self.get_logger().info(
                    f"Station {st.station_id}: regular={regular} "
                    f"batch_1080={b1080} batch_90={b90}"
                )
            if st.station_type in (StationMsg.ST_WORKBENCH, StationMsg.ST_HYBRID):
                if workbench_station_id is None:
                    workbench_station_id = st.station_id
            if st.station_type == StationMsg.ST_CUSTOMER:
                customer_station_id = st.station_id
                customer_stations.append(st)

        if workbench_station_id is None:
            raise RuntimeError("No workbench station in arena layout")
        if customer_station_id is None:
            raise RuntimeError("No customer station in arena layout")

        home_id = 0

        # Compute aidlist and net_aidlist (recycling already subtracted)
        aidlist, net_aidlist, recycled_materials = compute_net_aidlist(
            produce_ids, recycle_ids
        )
        self.get_logger().info(
            f"aidlist={dict(aidlist)}, net_aidlist={dict(net_aidlist)}"
        )

        # Simulate Phase 1 recycle disassembly to detect cargo overflow at plan time.
        # Materials that would overflow and are still needed get added back to
        # net_aidlist so they are fetched from storage instead of lost silently.
        if recycle_ids:
            overflow_needed = self._check_recycle_overflow(recycle_ids, net_aidlist)
            if overflow_needed:
                net_aidlist += overflow_needed
                self.get_logger().info(
                    f"net_aidlist after recycle overflow correction: {dict(net_aidlist)}"
                )

        # Step A: build storage midlist and check if it alone satisfies net_aidlist
        if self._calc:
            storage_mid = build_storage_midlist(storage_stations, self._calc, home_id)
        else:
            storage_mid = [
                {'station_id': s['station_id'], 'materials': s['material_ids'],
                 'distance': 0.0, 'is_recycle_pickup': False, 'recycle_product_id': None}
                for s in storage_stations
            ]

        satisfied, missing = check_storage_satisfies(storage_mid, net_aidlist)

        # Step B: if not satisfied, add batch 10-80 and re-check
        use_batch_1080 = False
        if not satisfied and batch_stations_1080:
            use_batch_1080 = True
            if self._calc:
                bidlist_1080 = build_bidlist(batch_stations_1080, self._calc, home_id)
            else:
                bidlist_1080 = [
                    {
                        'station_id': bst['station_id'],
                        'materials': [
                            BATCH_TO_MATERIAL[bid]
                            for bid in bst['batch_ids']
                            for _ in range(BATCH_COUNT)
                        ],
                        'distance': 0.0,
                        'is_recycle_pickup': False,
                        'recycle_product_id': None,
                        'is_batch': True,
                        'is_mix_batch': False,
                    }
                    for bst in batch_stations_1080
                ]
            merged_check = merge_into_midlist(storage_mid, bidlist_1080)
            satisfied, missing = check_storage_satisfies(merged_check, net_aidlist)
            self.get_logger().info(
                f"After batch 10-80: satisfied={satisfied}, missing={dict(missing)}"
            )

        # Step C: if still missing (after storage + batch_1080 + recycling already in net_aidlist),
        # assign mix batch 90 to cover remaining shortage
        missing_for_mix = missing if (not satisfied and batch_stations_90) else None
        if missing_for_mix:
            self.get_logger().info(
                f"Mix batch 90 assigned for: {dict(missing_for_mix)}"
            )

        if not satisfied and not missing_for_mix:
            self.get_logger().warning(
                f"Cannot satisfy aidlist — missing: {dict(missing)}"
            )

        # Recycling is always triggered when recycle orders exist
        needs_recycling = bool(recycle_ids)

        # Build recycle orders (map each recycle product to the customer station)
        recycle_orders = [
            {'station_id': customer_station_id, 'product_id': pid}
            for pid in recycle_ids
        ]

        # Build full midlist with batch support
        if self._calc:
            full_midlist = build_full_midlist(
                storage_stations=storage_stations,
                customer_stations=[],
                recycle_orders=recycle_orders,
                calc=self._calc,
                home_station_id=home_id,
                workbench_station_id=workbench_station_id,
                needs_recycling=needs_recycling,
                batch_stations_1080=batch_stations_1080 if use_batch_1080 else None,
                batch_stations_90=batch_stations_90 if missing_for_mix else None,
                missing_for_mix=missing_for_mix,
            )
        else:
            full_midlist = [
                {'station_id': o['station_id'], 'materials': [],
                 'distance': 0.0, 'is_recycle_pickup': True,
                 'recycle_product_id': o['product_id']}
                for o in recycle_orders
            ] + storage_mid
            if use_batch_1080:
                for bst in batch_stations_1080:
                    mats = [
                        BATCH_TO_MATERIAL[bid]
                        for bid in bst['batch_ids']
                        for _ in range(BATCH_COUNT)
                    ]
                    full_midlist.append({
                        'station_id': bst['station_id'],
                        'materials': mats,
                        'distance': float('inf'),
                        'is_recycle_pickup': False,
                        'recycle_product_id': None,
                        'is_batch': True,
                        'is_mix_batch': False,
                    })

        # Build final mid list
        mid = build_mid(full_midlist, net_aidlist)

        # Determine which products go to workbench vs in-transit
        temp_alloc = CargoAllocator()
        intransit_allocated = temp_alloc.allocate(produce_ids)
        intransit_ids = list(intransit_allocated.keys())
        workbench_ids = [
            pid for pid in produce_ids if pid not in intransit_ids
        ]

        # Surplus recycled materials (obtained beyond what net_aidlist needs)
        surplus = {}
        for mat, cnt in recycled_materials.items():
            extra = cnt - (aidlist.get(mat, 0))
            if extra > 0:
                surplus[mat] = extra

        plan = Plan(
            mid=mid,
            workbench_products=workbench_ids,
            intransit_products=intransit_ids,
            workbench_station_id=workbench_station_id,
            customer_station_id=customer_station_id,
            home_station_id=home_id,
            surplus_recycled=surplus,
        )

        self.get_logger().info(
            f"Plan ready: {len(mid)} pickup entries, "
            f"workbench={workbench_ids}, in-transit={intransit_ids}"
        )
        return plan

    def _check_recycle_overflow(
        self, recycle_ids: list, net_aidlist: Counter
    ) -> Counter:
        """Simulate Phase 1 cargo loading for all recycled products.

        Returns a Counter of materials that would overflow cargo slots 2-6
        AND are still needed (i.e., present in net_aidlist).  Callers should
        add the returned Counter to net_aidlist so those materials are picked
        from storage in Phase 2 instead of being silently lost.
        """
        from robocup_planner.execution.cargo_state import CargoManager
        from robocup_planner.product_catalog import get_material_count

        sim = CargoManager()
        overflow: Counter = Counter()
        for pid in recycle_ids:
            for mat_id, cnt in get_material_count(pid).items():
                for _ in range(cnt):
                    if sim.place_material(mat_id) is None:
                        overflow[mat_id] += 1

        if not overflow:
            return Counter()

        overflow_needed: Counter = Counter()
        for mat_id, cnt in overflow.items():
            recoverable = min(cnt, net_aidlist.get(mat_id, 0))
            if recoverable > 0:
                overflow_needed[mat_id] = recoverable

        total_overflow = sum(overflow.values())
        total_needed = sum(overflow_needed.values())
        self.get_logger().warning(
            f"[PLAN] Recycle cargo overflow detected: {total_overflow} block(s) "
            f"would overflow {dict(overflow)}; {total_needed} needed — "
            "routing overflowed needed materials to storage pickup"
        )
        return overflow_needed

    # ------------------------------------------------------------------
    # Blocking helpers called by Executor (run in executor thread)
    # ------------------------------------------------------------------

    def navigate(self, station_id: int) -> bool:
        """Navigate directly to station_id goal (positive) or sub_goal (negative)."""
        self._nav_client.wait_for_server()

        done = threading.Event()
        success_holder = [False]

        def _result_cb(future):
            result = future.result()
            success_holder[0] = result.result.success
            done.set()

        def _goal_cb(future):
            gh = future.result()
            if not gh.accepted:
                self.get_logger().error(f"NavTask goal rejected for station {station_id}")
                done.set()
                return
            gh.get_result_async().add_done_callback(_result_cb)

        goal = NavTask.Goal()
        goal.station_id = station_id
        self._nav_client.send_goal_async(goal).add_done_callback(_goal_cb)
        done.wait()

        if not success_holder[0]:
            self.get_logger().error(f"Navigation to station {station_id} failed")
        return success_holder[0]

    def call_post_process(self) -> bool:
        """Trigger post-process exit maneuver (backup + rotate) after docking.

        Calls /robocup_navigator/post_process.  Returns True on success or if
        there was nothing pending (navigator responds NO_PENDING_POST_PROCESS).
        """
        if not self._post_process_client.wait_for_service(timeout_sec=2.0):
            self.get_logger().warning("[POST] post_process service unavailable")
            return False

        future = self._post_process_client.call_async(Trigger.Request())
        done = threading.Event()
        result_holder = [None]

        def _cb(f):
            result_holder[0] = f.result()
            done.set()

        future.add_done_callback(_cb)
        done.wait()

        resp = result_holder[0]
        if resp is None or not resp.success:
            msg = resp.message if resp else 'no response'
            self.get_logger().warning(f"[POST] post_process failed: {msg}")
            return False

        self.get_logger().info(
            f"[POST] post_process OK"
            + (f": {resp.message}" if resp.message else "")
        )
        return True

    def navigate_subgoal(self, station_id: int) -> bool:
        """Navigate to the sub_goal (approach) position of station_id.

        Convention: negative station_id signals the nav server to stop at
        the sub_goal waypoint (station_N_sub_goal) instead of the docking goal.
        """
        self.get_logger().info(f"[NAV] → sub_goal of station {station_id}")
        return self.navigate(-abs(station_id))

    def navigate_goal(self, station_id: int) -> bool:
        """Navigate the final leg from sub_goal to the docking goal of station_id.

        No arm assembly should occur during this phase (precision parking).
        """
        self.get_logger().info(f"[NAV] → goal of station {station_id}")
        return self.navigate(abs(station_id))

    def arm_assemble_intransit_async(self, product_id: int, cargo_id: int) -> None:
        """Start in-transit assembly on cargo_id asynchronously.

        The arm stacks blocks on cargo 7/8 while the AMR moves.  Callers must
        invoke wait_for_intransit_assembly() before navigate_goal() to ensure
        the arm is idle during the precision-parking phase.
        """
        event = threading.Event()
        with self._intransit_lock:
            self._intransit_events.append(event)

        def _assemble():
            self.get_logger().info(
                f"[ARM] ASSEMBLE product={product_id} cargo={cargo_id}"
            )
            success = self._arm_call(
                'ASSEMBLE',
                object_ids=[product_id],
                location=cargo_id,
                station_id=cargo_id,
            )
            if not success:
                self.get_logger().warning(
                    f"[ARM] ASSEMBLE failed: product={product_id}"
                )
            event.set()

        threading.Thread(
            target=_assemble, daemon=True, name=f'assemble_{product_id}'
        ).start()

    def wait_for_intransit_assembly(self) -> None:
        """Block until all pending in-transit ASSEMBLE operations finish.

        Call this at sub_goal before navigate_goal() so the arm is idle
        during the sub_goal → goal precision-parking segment.
        """
        with self._intransit_lock:
            events = list(self._intransit_events)
            self._intransit_events.clear()

        if events:
            self.get_logger().info(
                f"[ARM] Waiting for {len(events)} in-transit assembly operation(s)"
            )
            for ev in events:
                ev.wait()
            self.get_logger().info("[ARM] All in-transit assemblies complete")

    def wb_task(self, work_type: str, product_id: int) -> bool:
        """Block until the workbench completes the requested work."""
        self._wb_client.wait_for_server()

        done = threading.Event()
        success_holder = [False]

        def _result_cb(future):
            success_holder[0] = future.result().result.success
            done.set()

        def _goal_cb(future):
            gh = future.result()
            if not gh.accepted:
                self.get_logger().error(f"WbTask goal rejected ({work_type} {product_id})")
                done.set()
                return
            gh.get_result_async().add_done_callback(_result_cb)

        goal = WbTask.Goal()
        goal.work_type = work_type
        goal.product_id = product_id
        self._wb_client.send_goal_async(goal).add_done_callback(_goal_cb)
        done.wait()
        return success_holder[0]

    def _arm_call(
        self,
        action: str,
        object_ids: list,
        location: int = 0,
        station_id: int = None,
    ) -> bool:
        """Send one ArmCommand service call to the arm. Blocks until response."""
        self._arm_client.wait_for_service()

        req = ArmCommand.Request()
        req.action = action
        req.object_ids = [int(x) for x in object_ids]
        req.location = int(location)
        req.station_id = int(station_id if station_id is not None else location)

        future = self._arm_client.call_async(req)
        done = threading.Event()

        def _cb(f):
            done.set()

        future.add_done_callback(_cb)
        done.wait()
        return future.result().success

    def arm_pick_material(self, station_id: int, material_id: int) -> bool:
        """Pick one material block from a storage station and place it on cargo."""
        return self._arm_call(
            ARM_PICK,
            object_ids=[material_id],
            location=station_id,
            station_id=station_id,
        )

    def arm_pick_product(self, station_id: int, product_id: int) -> bool:
        """Pick an assembled product from a customer counter (for recycling)."""
        return self._arm_call(
            ARM_PICK,
            object_ids=[product_id],
            location=station_id,
            station_id=station_id,
        )

    def arm_unload_material(self, object_id: int) -> bool:
        """Unload a material block from cargo to the workbench.
        The arm locates the block via cargo_manager FIND_OBJECT."""
        return self._arm_call(
            ARM_PLACE,
            object_ids=[object_id],
            location=0,
        )

    def arm_deliver(self, product_id: int, from_cargo_id: int = 0) -> bool:
        """Deliver a finished product to the customer counter.
        The arm locates the product via cargo_manager FIND_OBJECT."""
        self.get_logger().info(
            f"[ARM] deliver product_id={product_id} from cargo {from_cargo_id}"
        )
        return self._arm_call(
            ARM_DELIVER,
            object_ids=[product_id],
            location=0,
        )


# ------------------------------------------------------------------
# Entry point
# ------------------------------------------------------------------

def main(args=None):
    rclpy.init(args=args)
    node = PlannerNode()

    # MultiThreadedExecutor lets action/service callbacks run while
    # the executor thread is blocking inside navigate() / wb_task().
    ros_executor = MultiThreadedExecutor(num_threads=4)
    ros_executor.add_node(node)

    try:
        ros_executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
