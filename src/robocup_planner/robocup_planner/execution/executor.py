"""
Hybrid executor: follows a static mid list while reacting to workbench
signals and in-transit assembly completions at runtime.

Workbench interrupt rule (M4):
  - AMR in transit      → divert after completing the NEXT station visit.
  - AMR at a station    → divert after the current load/unload finishes.
  - AMR already en route to workbench → ignore the signal (debounce).

Cargo overflow rule (A1):
  If all cargo 2-6 slots are full and no workbench assembly is ready yet,
  go to the workbench anyway to drop partial materials as a buffer.

In-transit assembly rule (M1 clarification):
  Completed products on cargo 7/8 are delivered directly from cargo 7/8
  to the customer counter — they do NOT move to cargo 1 first.

Two-phase navigation rule (sub_goal / goal):
  Every station approach is split into two legs:
    1. navigate_subgoal(id)  — fast cruise to the approach waypoint.
       At sub_goal: wait_for_intransit_assembly() blocks until the arm
       finishes any ASSEMBLE operation started during the previous leg.
    2. navigate_goal(id)     — slow precision parking to the docking point.
       No arm assembly commands are issued during this phase.

  When delivering to the customer counter, if in-transit cargo-7/8
  assembly is still running on arrival at sub_goal, the executor waits
  there until the arm finishes, then proceeds to goal.
"""

import threading
from dataclasses import dataclass, field
from typing import Dict, List, Optional

from robocup_planner.execution.cargo_state import CargoManager
from robocup_planner.planning.cargo_allocator import CargoAllocator
from robocup_planner.product_catalog import get_material_count


@dataclass
class Plan:
    """Output of the planning phase, consumed by the Executor."""
    mid: List[Dict]                   # ordered pickup sequence (build_mid output)
    workbench_products: List[int]     # product IDs that require the workbench
    intransit_products: List[int]     # product IDs allocated to cargo 7/8
    workbench_station_id: int
    customer_station_id: int
    home_station_id: int
    # Surplus recycled materials (not needed by any assembly; may arrive on cargo
    # after Phase 1 recycle disassembly — tracked for overflow awareness).
    surplus_recycled: Dict[int, int] = field(default_factory=dict)


class Executor:
    """
    Runs the reactive execution loop. Calls blocking helper methods on
    the PlannerNode (navigate, arm_pick, arm_unload, wb_task, arm_deliver).
    Must run in a dedicated thread while the ROS2 node spins separately.
    """

    def __init__(self, plan: Plan, node):
        self._plan = plan
        self._node = node
        self._cargo = CargoManager()
        self._allocator = CargoAllocator()
        self._allocator.allocate(plan.intransit_products)

        # Workbench signal: set by PlannerNode callback when /workbench/product_ready fires.
        self.wb_signal = threading.Event()
        # Remaining workbench-only products not yet assembled.
        self._pending_wb: List[int] = list(plan.workbench_products)
        # Count of deliverables ready (cargo 1 + assembled cargo 7/8 slots).
        self._pending_deliveries: int = 0
        # mid_cursor persists across workbench detours (C1 fix).
        self._mid_cursor: int = 0
        # Prevent re-entrant workbench trips.
        self._en_route_to_wb: bool = False
        # FIFO queue of product_ids assembled at workbench and waiting on cargo 1.
        self._cargo1_queue: List[int] = []
        # In-transit product_ids whose ASSEMBLE command has already been started.
        self._started_intransit: set[int] = set()

    # ------------------------------------------------------------------
    # Public entry point
    # ------------------------------------------------------------------

    def run(self) -> None:
        self._log("Executor started")

        # Phase 1: recycle pickups (customer counter → workbench → disassembly)
        self._run_recycle_phase()

        # After Phase 1: reclaimed materials may already satisfy a workbench product.
        if self._should_go_workbench():
            self._divert_to_workbench()
        if self._should_deliver():
            self._deliver_all()

        # Phase 2: main pickup loop
        self._run_pickup_phase()

        # Final delivery of anything remaining
        self._deliver_all()

        self._log("Executor finished")

    # ------------------------------------------------------------------
    # Phase 1 — Recycle pickup
    # ------------------------------------------------------------------

    def _run_recycle_phase(self) -> None:
        recycle_entries = [
            e for e in self._plan.mid if e['is_recycle_pickup']
        ]
        if not recycle_entries:
            return

        self._log(f"Phase 1: collecting {len(recycle_entries)} recycled product(s)")

        # Collect all products from each customer counter in one visit.
        entries_by_station = {}
        for entry in recycle_entries:
            entries_by_station.setdefault(entry['station_id'], []).append(entry)

        for station_id, entries in entries_by_station.items():
            product_ids = [e['recycle_product_id'] for e in entries]
            self._log(f"  → navigate to customer station {station_id} (sub_goal)")
            self._node.navigate_subgoal(station_id)
            self._wait_for_intransit_assembly()
            self._node.navigate_goal(station_id)
            self._log(f"  → pick product(s) {product_ids}")
            self._node.arm_pick_products(
                station_id=station_id,
                product_ids=product_ids,
            )
            self._node.call_post_process()

        # Bring all collected products to the workbench for disassembly.
        sid = self._plan.workbench_station_id
        self._log(f"  → navigate to workbench {sid} (sub_goal)")
        self._node.navigate_subgoal(sid)
        self._wait_for_intransit_assembly()
        self._node.navigate_goal(sid)

        for entry in recycle_entries:
            pid = entry['recycle_product_id']
            self._log(f"  → workbench disassembly of product {pid}")
            self._node.wb_task('RECYCLE', pid)
            # After disassembly the arm places reclaimed materials onto cargo 2-6.
            # We update cargo state to reflect the incoming materials.
            reclaimed = get_material_count(pid)
            for mat_id, count in reclaimed.items():
                for _ in range(count):
                    placement = self._cargo.place_material(mat_id)
                    if placement is None:
                        self._log(
                            f"  ! cargo full during recycle unload of mat {mat_id}; "
                            "block lost (planner should have routed this to storage pickup)"
                        )
        self._node.call_post_process()

    # ------------------------------------------------------------------
    # Phase 2 — Main pickup loop
    # ------------------------------------------------------------------

    def _run_pickup_phase(self) -> None:
        storage_entries = [
            e for e in self._plan.mid if not e['is_recycle_pickup']
        ]
        self._mid_cursor = 0

        while self._mid_cursor < len(storage_entries):
            entry = storage_entries[self._mid_cursor]
            sid = entry['station_id']
            self._log(
                f"[{self._mid_cursor}/{len(storage_entries)-1}] "
                f"navigate to storage station {sid}"
            )

            # Two-phase approach: cruise to sub_goal (arm may still assemble),
            # then wait for assembly, then precision-park at goal (arm idle).
            self._node.navigate_subgoal(sid)
            self._wait_for_intransit_assembly()
            self._node.navigate_goal(sid)

            # Pick each required material from this station.
            for mat_id in entry['pickup_materials']:
                cargo_id = self._decide_placement(mat_id)

                if cargo_id is None:
                    # Cargo 2-6 full and no material fits — divert to workbench now.
                    self._log("  ! cargo full — diverting to workbench before pickup")
                    self._node.call_post_process()
                    self._divert_to_workbench(forced=True)
                    cargo_id = self._decide_placement(mat_id)
                    if cargo_id is None:
                        self._log(f"  !! still no space for mat {mat_id} — skipping")
                        continue
                    # Re-navigate to the storage station after the workbench detour.
                    self._node.navigate_subgoal(sid)
                    self._wait_for_intransit_assembly()
                    self._node.navigate_goal(sid)

                self._log(f"  → pick mat {mat_id} → cargo {cargo_id}")
                self._node.arm_pick_material(
                    station_id=sid,
                    material_id=mat_id,
                )
                self._on_material_placed(mat_id, cargo_id)

            self._node.call_post_process()
            self._mid_cursor += 1

            # Post-station checks (order: workbench first, then delivery).
            if self._should_go_workbench():
                self._divert_to_workbench()

            if self._should_deliver():
                self._deliver_all()

    # ------------------------------------------------------------------
    # Cargo placement decision
    # ------------------------------------------------------------------

    def _decide_placement(self, material_id: int) -> Optional[int]:
        """
        Returns the cargo_id (2-8) where material_id will be placed, or None
        if no space is available anywhere.

        Priority:
          1. Cargo 2-6 first slot with enough stack space.

        The arm LOAD implementation chooses from material slots 2-6.  Cargo
        7/8 are target slots for the later ASSEMBLE command, not raw-material
        load destinations.
        """
        return self._cargo.place_material(material_id)

    def _on_material_placed(self, material_id: int, cargo_id: int) -> None:
        """Update state after a block is successfully placed."""
        self._start_ready_intransit_assembly()

    def _start_ready_intransit_assembly(self) -> bool:
        """
        Start one ready in-transit assembly, if possible.

        Raw materials are loaded into cargo 2-6.  Once all materials for an
        allocated in-transit product are present, ASSEMBLE moves those materials
        onto the product's target slot (cargo 7/8).  Only one assembly is
        started per call because the AMR has a single arm.
        """
        for product_id in self._plan.intransit_products:
            if product_id in self._started_intransit:
                continue

            slots = self._cargo.find_materials_for_product(product_id)
            if slots is None:
                continue

            cargo_id = self._allocator.mark_assembled(product_id)
            if cargo_id is None:
                continue

            for material_cargo_id, mat_id in slots:
                self._cargo.remove_material(material_cargo_id, mat_id)

            self._started_intransit.add(product_id)
            self._log(
                f"  [READY] in-transit product {product_id} → cargo {cargo_id}; "
                "starting ASSEMBLE async"
            )
            self._node.arm_assemble_intransit_async(product_id, cargo_id)
            self._pending_deliveries += 1
            return True

        return False

    def _wait_for_intransit_assembly(self) -> None:
        """
        Wait until the arm is idle before the precision goal leg.

        If more products became ready while one assembly was running, start and
        drain them here so sub_goal → goal remains arm-idle.
        """
        while True:
            self._node.wait_for_intransit_assembly()
            if not self._start_ready_intransit_assembly():
                return

    # ------------------------------------------------------------------
    # Workbench divert
    # ------------------------------------------------------------------

    def _should_go_workbench(self) -> bool:
        if self._en_route_to_wb:
            return False
        if self.wb_signal.is_set():
            return True
        # Check if any pending workbench product has all its materials ready.
        return self._cargo.can_assemble_for_workbench(self._pending_wb) is not None

    def _divert_to_workbench(self, forced: bool = False) -> None:
        """
        Navigate to the workbench, unload materials for one product,
        start workbench assembly, then return.  The mid_cursor is preserved
        so pickup resumes from the same station (C1 fix).
        """
        if self._en_route_to_wb and not forced:
            return

        self._en_route_to_wb = True
        self.wb_signal.clear()

        wid = self._plan.workbench_station_id
        self._log(f"  → divert to workbench {wid}")
        self._node.navigate_subgoal(wid)
        self._wait_for_intransit_assembly()
        self._node.navigate_goal(wid)

        ready_pid = self._cargo.can_assemble_for_workbench(self._pending_wb)
        if ready_pid is not None:
            self._unload_product_materials(ready_pid)
            self._log(f"  → workbench assemble product {ready_pid}")
            self._node.wb_task('PRODUCE', ready_pid)
            self._pending_wb.remove(ready_pid)
            self._cargo.add_finished_product()
            self._cargo1_queue.append(ready_pid)
            self._pending_deliveries += 1
        elif forced:
            # Overflow drop: unload whatever is on cargo to free space.
            self._log("  → overflow drop: unloading all cargo to workbench")
            self._unload_all_materials()

        self._node.call_post_process()
        self._en_route_to_wb = False

    def _unload_product_materials(self, product_id: int) -> None:
        """Remove materials for product_id from cargo 2-6 (arm drops them at workbench)."""
        slots = self._cargo.find_materials_for_product(product_id)
        if slots is None:
            return
        for cargo_id, mat_id in slots:
            self._node.arm_unload_material(mat_id)
            self._cargo.remove_material(cargo_id, mat_id)

    def _unload_all_materials(self) -> None:
        """Drop everything in cargo 2-6 at the workbench (overflow buffer)."""
        for cargo_id, mat_id in list(self._cargo.all_materials()):
            self._node.arm_unload_material(mat_id)
            self._cargo.remove_material(cargo_id, mat_id)

    # ------------------------------------------------------------------
    # Delivery
    # ------------------------------------------------------------------

    def _should_deliver(self) -> bool:
        return self._pending_deliveries > 0

    def _deliver_all(self) -> None:
        if not self._should_deliver():
            return

        cid = self._plan.customer_station_id

        # Navigate to sub_goal first: if in-transit assembly is still running
        # (arm hasn't finished ASSEMBLE on cargo 7/8), wait here before the
        # precision-parking approach — satisfying the sub_goal hold requirement.
        self._log(f"  → navigate to customer {cid} (sub_goal)")
        self._node.navigate_subgoal(cid)
        self._log("  → waiting for in-transit assembly at sub_goal (if any)")
        self._wait_for_intransit_assembly()
        # Final approach — arm is idle during sub_goal → goal phase.
        self._node.navigate_goal(cid)

        # Deliver products from cargo 1 (assembled at workbench).
        while self._cargo.finished_on_cargo1 > 0:
            product_id = self._cargo1_queue.pop(0) if self._cargo1_queue else 0
            self._log(f"  → deliver product {product_id} from cargo 1")
            self._node.arm_deliver(product_id=product_id, from_cargo_id=1)
            self._cargo.consume_finished_product()
            self._pending_deliveries -= 1

        # Deliver in-transit assembled products directly from cargo 7/8 (M1 clarification).
        for slot in list(self._allocator.get_completed_slots()):
            self._log(
                f"  → deliver in-transit product {slot.product_id} from cargo {slot.cargo_id}"
            )
            self._node.arm_deliver(product_id=slot.product_id, from_cargo_id=slot.cargo_id)
            self._allocator.free_slot(slot.cargo_id)
            self._pending_deliveries -= 1

        self._node.call_post_process()

    # ------------------------------------------------------------------
    # Utility
    # ------------------------------------------------------------------

    def _log(self, msg: str) -> None:
        self._node.get_logger().info(f"[Executor] {msg}")
