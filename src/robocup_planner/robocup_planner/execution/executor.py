"""
Hybrid executor: follows a static mid list while reacting to in-transit
assembly completions at runtime.

Cargo arm assembly rule (Mod 3):
  All products — including multi-layer products (Burger, Big Tree, Ice Cream) —
  are assembled by the AMR cargo arm on cargo slots 7/8.
  The workbench is used ONLY for RECYCLE (disassembly) in Phase 1.
  No wb_task('PRODUCE') calls are ever issued.

Cargo overflow rule:
  If all cargo 2-6 slots are full and no in-transit slot is assembling yet,
  go to the workbench to drop materials as a buffer.

Recycle disassembly rule (Phase 1):
  RECYCLE runs on the workbench asynchronously (wb_task_async), so the AMR
  drives off to fetch the next recycle product instead of idling at the
  workbench while it disassembles. Materials from the previous disassembly
  are always collected before the next product is placed on the workbench
  shelf. If cargo 2-6 fills up while multiple recycle products are in
  flight, the load is dropped at the workbench before heading to the next
  pickup instead of risking an overflow trip mid-route.

In-transit assembly rule (Mod 2):
  Completed products on cargo 7/8 are delivered directly from cargo 7/8
  to the customer counter — they do NOT move to cargo 1 first.
  When a slot is freed after delivery, CargoAllocator auto-assigns the next
  queued product to that slot; _start_ready_intransit_assembly() is called
  after the customer post-process exit maneuver so assembly does not overlap
  with backup/rotation.

Two-phase navigation rule:
  Every station approach is split into two legs:
    1. navigate_subgoal(id)  — fast cruise; arm may ASSEMBLE during this leg.
       At sub_goal: wait_for_intransit_assembly() blocks until arm is idle.
    2. navigate_goal(id)     — precision parking; arm must be idle.
"""

from dataclasses import dataclass, field
from typing import Dict, List, Optional

from robocup_planner.planning.cargo_allocator import CargoAllocator
from robocup_planner.product_catalog import get_material_count


@dataclass
class Plan:
    """Output of the planning phase, consumed by the Executor."""
    mid: List[Dict]                   # ordered pickup sequence (build_mid output)
    workbench_products: List[int]     # always empty (Mod 3); kept for compat
    intransit_products: List[int]     # ALL product IDs queued for cargo arm assembly
    workbench_station_id: int
    customer_station_id: int
    home_station_id: int
    surplus_recycled: Dict[int, int] = field(default_factory=dict)
    material_home_station: Dict[int, int] = field(default_factory=dict)
    product_weights: Dict[int, float] = field(default_factory=dict)


class ExecutionFailure(RuntimeError):
    """Raised when a required runtime action fails and the plan cannot continue."""


class Executor:
    """
    Runs the reactive execution loop. Calls blocking helper methods on
    the PlannerNode (navigate, arm_pick, arm_unload, wb_task, arm_deliver).
    Must run in a dedicated thread while the ROS2 node spins separately.
    """

    def __init__(self, plan: Plan, node):
        self._plan = plan
        self._node = node
        self._allocator = CargoAllocator()
        self._allocator.allocate(plan.intransit_products, plan.product_weights)

        # Cargo IDs whose ASSEMBLE command has already been started.
        # Product IDs can repeat in one order, so slot state is keyed by cargo.
        self._started_intransit: set = set()
        # Prevent re-entrant overflow trips.
        self._en_route_to_wb: bool = False

    # ------------------------------------------------------------------
    # Public entry point
    # ------------------------------------------------------------------

    def run(self) -> None:
        self._log("Executor started")

        # Phase 1: recycle pickups (customer counter → workbench → disassembly)
        self._run_recycle_phase()

        # Return recycled materials that exceed what the order needs to their
        # designated storage station, instead of leaving them stuck in cargo.
        self._return_surplus_materials()

        # After Phase 1: reclaimed materials may already satisfy a queued product.
        self._start_ready_intransit_assembly()
        self._wait_for_intransit_assembly()
        if self._has_ready_deliveries():
            self._deliver_all()

        # Phase 2: main pickup loop
        self._run_pickup_phase()

        # Final delivery of anything remaining
        self._wait_for_intransit_assembly()
        self._deliver_all()

        # Return to home station (0 for A-side, 14 for B-side)
        self._return_to_home()

        self._log("Executor finished")

    # ------------------------------------------------------------------
    # Phase 1 — Recycle pickup
    # ------------------------------------------------------------------

    def _run_recycle_phase(self) -> None:
        """Pick up every recycle product and disassemble it at the workbench.

        RECYCLE runs asynchronously on the workbench so the AMR can drive off
        to fetch the next recycle product instead of idling next to the
        workbench while it disassembles. To avoid stacking a new product on
        top of an unclaimed pile of material blocks, materials from the
        previous disassembly are always collected before the next product is
        placed on the workbench. If cargo 2-6 fills up (typically once 3+
        recycle products' worth of material has accumulated), the load is
        dropped at the workbench before the AMR sets off for the next pickup.
        """
        recycle_entries = [
            e for e in self._plan.mid if e.get('is_recycle_pickup')
        ]
        if not recycle_entries:
            return

        self._log(f"Phase 1: recycling {len(recycle_entries)} product(s)")
        workbench_id = self._plan.workbench_station_id

        pending_wb_handle = None
        pending_wb_pid = None

        for idx, entry in enumerate(recycle_entries):
            station_id = entry['station_id']
            pid = entry['recycle_product_id']

            self._log(f"  → navigate to customer station {station_id}")
            self._node.navigate_subgoal(station_id)
            self._wait_for_intransit_assembly()
            self._node.navigate_goal(station_id)
            self._node.arm_pick_product(station_id=station_id, product_id=pid)
            self._node.call_post_process()

            self._log(f"  → navigate to workbench {workbench_id} for RECYCLE")
            self._node.navigate_goal(workbench_id)

            # Clear out the previous product's disassembled materials first so
            # the workbench shelf is free before we place the next product on it.
            if pending_wb_handle is not None:
                self._collect_recycled_materials(
                    pending_wb_handle, pending_wb_pid, workbench_id
                )
                pending_wb_handle = None
                pending_wb_pid = None

            self._node.arm_unload_product_to_workbench(
                product_id=pid,
                station_id=workbench_id,
            )
            pending_wb_handle = self._node.wb_task_async('RECYCLE', pid)
            pending_wb_pid = pid
            self._node.call_post_process()
            self._start_ready_intransit_assembly()

            # Cargo pressure check: with several recycle products in flight,
            # cargo 2-6 can fill up with reclaimed material before it's all
            # needed. Drop the load here (we're already at the workbench)
            # rather than risking an overflow trip mid-route to the next pickup.
            is_last = (idx == len(recycle_entries) - 1)
            if not is_last and self._node.cargo_is_full():
                self._log(
                    "  ! cargo full during recycle phase — "
                    "dropping materials at workbench before next pickup"
                )
                self._node.arm_unload_all_materials()
                self._node.call_post_process()

        # Collect whatever disassembly is still pending after the last pickup.
        if pending_wb_handle is not None:
            self._collect_recycled_materials(
                pending_wb_handle, pending_wb_pid, workbench_id
            )
            self._start_ready_intransit_assembly()

    def _collect_recycled_materials(
        self, handle, pid: int, workbench_id: int
    ) -> None:
        """Wait for a RECYCLE WbTask to finish, then pick up every material
        block it produced from the workbench shelf."""
        if not self._node.wait_for_wb_task(handle):
            raise ExecutionFailure(f"RECYCLE failed: product={pid}")

        for mat_id, cnt in get_material_count(pid).items():
            for _ in range(cnt):
                self._log(f"    ← pick recycled mat {mat_id} from workbench")
                self._node.arm_pick_material(
                    station_id=workbench_id,
                    material_id=mat_id,
                )

    # ------------------------------------------------------------------
    # Return surplus recycled materials to their designated storage station
    # ------------------------------------------------------------------

    def _return_surplus_materials(self) -> None:
        """Return recycled materials beyond what's needed for production to
        their designated storage station.

        Blocks of the same material_id are fungible: the plan only tracks how
        many extra units exist, not which physical blocks, so any `count`
        units of that material currently in cargo can be dropped off.
        """
        surplus = {m: c for m, c in self._plan.surplus_recycled.items() if c > 0}
        if not surplus:
            return

        # Group by destination station so each station is visited once.
        by_station: Dict[int, List[int]] = {}
        for mat_id, cnt in surplus.items():
            station_id = self._plan.material_home_station.get(
                mat_id, self._plan.workbench_station_id
            )
            by_station.setdefault(station_id, []).extend([mat_id] * cnt)

        self._log(f"Returning surplus recycled materials: {surplus}")
        for station_id, mat_ids in by_station.items():
            self._log(f"  → navigate to storage {station_id} to return {mat_ids}")
            self._node.navigate_subgoal(station_id)
            self._wait_for_intransit_assembly()
            self._node.navigate_goal(station_id)
            for mat_id in mat_ids:
                self._node.arm_return_material_to_storage(
                    material_id=mat_id, station_id=station_id
                )
            self._node.call_post_process()

    # ------------------------------------------------------------------
    # Phase 2 — Main pickup loop
    # ------------------------------------------------------------------

    def _run_pickup_phase(self) -> None:
        storage_entries = [
            e for e in self._plan.mid if not e.get('is_recycle_pickup')
        ]

        for idx, entry in enumerate(storage_entries):
            sid = entry['station_id']
            self._log(
                f"[{idx}/{len(storage_entries)-1}] navigate to storage station {sid}"
            )

            self._node.navigate_subgoal(sid)
            self._wait_for_intransit_assembly()
            self._node.navigate_goal(sid)

            needs_revisit = False
            for mat_id in entry.get('pickup_materials', []):
                if self._node.cargo_is_full():
                    self._log("  ! cargo 2-6 full — overflow drop at workbench")
                    self._node.call_post_process()
                    self._overflow_drop_at_workbench()
                    # Must revisit the station after returning from workbench.
                    needs_revisit = True

                if needs_revisit:
                    self._node.navigate_subgoal(sid)
                    self._wait_for_intransit_assembly()
                    self._node.navigate_goal(sid)
                    needs_revisit = False

                self._log(f"  → pick mat {mat_id}")
                self._node.arm_pick_material(station_id=sid, material_id=mat_id)

            self._node.call_post_process()
            self._start_ready_intransit_assembly()

            if self._has_ready_deliveries():
                self._deliver_all()

    # ------------------------------------------------------------------
    # In-transit assembly
    # ------------------------------------------------------------------

    def _start_ready_intransit_assembly(self) -> bool:
        """Start ASSEMBLE for the first in-slot product whose materials are all loaded.

        Iterates cargo slots 7 and 8 so the higher-priority slot (assigned first
        by the allocator) is always checked first. Only one ASSEMBLE is started
        per call because the arm is single-threaded.
        Returns True if an ASSEMBLE was started.
        """
        for cargo_id in (7, 8):
            product_id = self._allocator.get_slot_product(cargo_id)
            if product_id is None or cargo_id in self._started_intransit:
                continue
            # Check if all required materials are available in cargo 2-6.
            if not self._node.cargo_has_all_materials(product_id):
                continue

            self._started_intransit.add(cargo_id)
            self._log(
                f"  [READY] in-transit product {product_id} → cargo {cargo_id}; "
                "starting ASSEMBLE async"
            )
            self._node.arm_assemble_intransit_async(product_id, cargo_id)
            return True

        return False

    def _wait_for_intransit_assembly(self) -> None:
        """Block until arm is idle; drain any newly-ready assemblies."""
        while True:
            completed = self._node.wait_for_intransit_assembly()
            for handle in completed:
                if not handle.success:
                    self._started_intransit.discard(handle.cargo_id)
                    raise ExecutionFailure(
                        f"ASSEMBLE failed: product={handle.product_id}, "
                        f"cargo={handle.cargo_id}"
                    )

                if not self._allocator.mark_slot_assembled(
                    handle.cargo_id, handle.product_id
                ):
                    raise ExecutionFailure(
                        f"ASSEMBLE state mismatch: product={handle.product_id}, "
                        f"cargo={handle.cargo_id}"
                    )
                self._log(
                    f"  [DONE] in-transit product {handle.product_id} "
                    f"assembled on cargo {handle.cargo_id}"
                )

            if not self._start_ready_intransit_assembly():
                return

    # ------------------------------------------------------------------
    # Overflow drop at workbench (cargo 2-6 full)
    # ------------------------------------------------------------------

    def _overflow_drop_at_workbench(self) -> None:
        """Navigate to workbench and unload all cargo 2-6 materials as a buffer."""
        if self._en_route_to_wb:
            return
        self._en_route_to_wb = True

        wid = self._plan.workbench_station_id
        self._log(f"  → overflow drop: navigate to workbench {wid}")
        self._node.navigate_subgoal(wid)
        self._wait_for_intransit_assembly()
        self._node.navigate_goal(wid)
        self._node.arm_unload_all_materials()
        self._node.call_post_process()

        self._en_route_to_wb = False

    # ------------------------------------------------------------------
    # Delivery
    # ------------------------------------------------------------------

    def _has_ready_deliveries(self) -> bool:
        return bool(self._allocator.get_completed_slots())

    def _deliver_all(self) -> None:
        completed = self._allocator.get_completed_slots()
        if not completed:
            return

        cid = self._plan.customer_station_id
        self._log(f"  → navigate to customer {cid}")
        self._node.navigate_subgoal(cid)
        self._wait_for_intransit_assembly()
        self._node.navigate_goal(cid)

        # Deliver each completed in-transit slot directly from cargo 7/8.
        queued_new_product = False
        for slot in list(self._allocator.get_completed_slots()):
            self._log(
                f"  → deliver product {slot.product_id} from cargo {slot.cargo_id}"
            )
            self._node.arm_deliver(
                product_id=slot.product_id,
                from_cargo_id=slot.cargo_id,
            )
            self._started_intransit.discard(slot.cargo_id)
            newly_queued = self._allocator.free_slot(slot.cargo_id)
            if newly_queued is not None:
                self._log(
                    f"  [QUEUE] product {newly_queued} now allocated to cargo {slot.cargo_id}"
                )
                queued_new_product = True

        self._node.call_post_process()
        if queued_new_product:
            self._start_ready_intransit_assembly()

    # ------------------------------------------------------------------
    # Return to home
    # ------------------------------------------------------------------

    def _return_to_home(self) -> None:
        """Navigate back to the home station (0 for A-side, 14 for B-side)."""
        home_id = self._plan.home_station_id
        self._log(f"Returning to home station {home_id}")
        self._node.navigate_subgoal(home_id)
        self._wait_for_intransit_assembly()
        self._node.navigate_goal(home_id)
        self._log(f"Arrived at home station {home_id} — mission complete")
        current = self._node.get_current_station_id()
        if current is not None and current != home_id:
            self._node.get_logger().error(
                f"[Executor] Home verification FAILED: "
                f"expected station {home_id}, navigator reports {current}"
            )
        else:
            self._log(f"Home verification OK: at station {home_id}")

    # ------------------------------------------------------------------
    # Utility
    # ------------------------------------------------------------------

    def _log(self, msg: str) -> None:
        self._node.get_logger().info(f"[Executor] {msg}")
