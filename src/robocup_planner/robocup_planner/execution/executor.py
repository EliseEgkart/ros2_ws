"""
Hybrid executor: follows a static mid list while reacting to in-transit
assembly completions at runtime.

Cargo arm assembly rule (Mod 3):
  All products — including multi-layer products (Burger, Big Tree, Ice Cream) —
  are assembled by the AMR cargo arm on cargo slots 7/8.
  The workbench is used ONLY for RECYCLE (disassembly) in Phase 1 / the
  deferred recycle phase. No wb_task('PRODUCE') calls are ever issued.

Cargo overflow rule:
  If all cargo 2-6 slots are full and no in-transit slot is assembling yet,
  go to the workbench to drop materials as a buffer. Since the workbench no
  longer consumes anything, dropped materials are tracked in self._wb_buffer
  and reclaimed at the next checkpoint (see _reclaim_workbench_buffer) —
  they are never simply abandoned there.

Recycle disassembly rule (Phase 1):
  RECYCLE runs on the workbench asynchronously (wb_task_async), so the AMR
  drives off to fetch the next recycle product instead of idling at the
  workbench while it disassembles. Materials from the previous disassembly
  are always collected before the next product is placed on the workbench
  shelf. If cargo 2-6 fills up while multiple recycle products are in
  flight, the load is dropped at the workbench before heading to the next
  pickup instead of risking an overflow trip mid-route. Cargo 1 occupancy
  (one recycled product in transit at a time) is enforced by CargoManager
  itself (see PlannerNode.arm_pick_product/arm_unload_product_to_workbench),
  not just by this call ordering.

Deferred recycle rule (lifecycle produce-then-recycle):
  A recycle order whose product_id has no matching stock on the customer
  counter at plan time can't be disassembled up front — the robot must
  assemble and deliver it first. Plan.deferred_recycle_ids lists those
  product_ids; _run_deferred_recycle_phase() reclaims them from the customer
  counter and disassembles them after every delivery is otherwise complete.
  CargoAllocator prioritizes these produce orders (see planner_node.py's
  deferred_recycle_priority_boost) so they clear the pipeline early instead
  of waiting on num_blocks/weight ordering alone.

In-transit assembly rule (Mod 2):
  Completed products on cargo 7/8 are delivered directly from cargo 7/8
  to the customer counter — they do NOT move to cargo 1 first.
  When a slot is freed after delivery, CargoAllocator auto-assigns the next
  queued product to that slot; _start_ready_intransit_assembly() is called
  immediately in case the new product's materials are already loaded.

Two-phase navigation rule:
  Every station approach is split into two legs, always via _approach(id):
    1. navigate_subgoal(id)  — fast cruise; arm may ASSEMBLE during this leg.
       At sub_goal: wait_for_intransit_assembly() blocks until arm is idle.
    2. navigate_goal(id)     — precision parking; arm must be idle.
  Never call navigate_subgoal()/navigate_goal() directly — always go
  through _approach() so this rule can't be silently skipped at a new
  call site (see _approach() docstring for why that happened before).

Fail-loud / self-recovery rule:
  Every navigate/arm/wb call the Executor makes is checked against its
  return value; a failure raises ExecutionFailure instead of silently
  continuing with a plan that no longer matches physical reality. run()
  catches any such failure, makes one best-effort attempt to return the AMR
  to its home station anyway (so a mid-mission fault never strands it in
  the arena), and then re-raises so the caller sees a clear error instead of
  a silently-incomplete or hung run.
"""

from collections import Counter
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
    # Product ids that are both produced and recycled this run, with no
    # matching stock on the customer counter at plan time. These must be
    # assembled, delivered, then picked back up for disassembly — handled by
    # Executor._run_deferred_recycle_phase() after normal delivery.
    deferred_recycle_ids: List[int] = field(default_factory=list)


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
        # Materials unloaded at the workbench as an overflow buffer, not yet
        # reclaimed. Since the workbench consumes nothing (Mod 3), anything
        # dropped here MUST be tracked and picked back up — see
        # _reclaim_workbench_buffer() and Known Issues #4.
        self._wb_buffer: Counter = Counter()

    # ------------------------------------------------------------------
    # Public entry point
    # ------------------------------------------------------------------

    def run(self) -> None:
        self._log("Executor started")

        try:
            # Phase 1: recycle pickups (customer counter → workbench → disassembly)
            self._run_recycle_phase()

            # Return recycled materials that exceed what the order needs to their
            # designated storage station, instead of leaving them stuck in cargo.
            self._return_surplus_materials()

            # Reclaim anything buffered at the workbench during Phase 1 before
            # cargo space gets contended again in Phase 2.
            self._reclaim_workbench_buffer()

            # After Phase 1: reclaimed materials may already satisfy a queued product.
            self._start_ready_intransit_assembly()
            self._wait_for_intransit_assembly()
            if self._has_ready_deliveries():
                self._deliver_all()

            # Phase 2: main pickup loop
            self._run_pickup_phase()

            # Reclaim anything buffered during Phase 2 overflow drops.
            self._reclaim_workbench_buffer()

            # Final delivery of anything remaining
            self._wait_for_intransit_assembly()
            self._deliver_all()

            # Lifecycle produce-then-recycle products: reclaim from the
            # customer counter now that they've been delivered, disassemble,
            # and return the materials to storage.
            self._run_deferred_recycle_phase()

            # One last safety net: nothing should still be buffered at this
            # point, but if it is (e.g. a reclaim attempt above hit a full
            # cargo and deferred itself), make a final attempt now rather
            # than silently finishing with material still stranded.
            self._reclaim_workbench_buffer()
        except Exception as e:
            self._log(
                f"! Execution failed mid-plan ({e!r}) — attempting best-effort "
                "return to home so the AMR isn't stranded"
            )
            try:
                self._return_to_home()
            except Exception as home_err:
                self._log(f"! Best-effort return-to-home also failed: {home_err!r}")
            raise

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
            self._approach(station_id)
            self._require(
                self._node.arm_pick_product(station_id=station_id, product_id=pid),
                f"arm_pick_product(station={station_id}, product={pid})",
            )
            self._soft(self._node.call_post_process(), "call_post_process (customer)")

            self._log(f"  → navigate to workbench {workbench_id} for RECYCLE")
            self._approach(workbench_id)

            # Clear out the previous product's disassembled materials first so
            # the workbench shelf is free before we place the next product on it.
            if pending_wb_handle is not None:
                self._collect_recycled_materials(
                    pending_wb_handle, pending_wb_pid, workbench_id
                )
                pending_wb_handle = None
                pending_wb_pid = None

            self._require(
                self._node.arm_unload_product_to_workbench(
                    product_id=pid, station_id=workbench_id,
                ),
                f"arm_unload_product_to_workbench(product={pid}, station={workbench_id})",
            )
            pending_wb_handle = self._node.wb_task_async('RECYCLE', pid)
            pending_wb_pid = pid
            self._soft(self._node.call_post_process(), "call_post_process (workbench)")

            # Cargo pressure check: with several recycle products in flight,
            # cargo 2-6 can fill up with reclaimed material before it's all
            # needed. Drop the load here (we're already at the workbench)
            # rather than risking an overflow trip mid-route to the next pickup.
            # The dropped materials are tracked in self._wb_buffer and
            # reclaimed at the next checkpoint — never silently abandoned.
            is_last = (idx == len(recycle_entries) - 1)
            if not is_last and self._node.cargo_is_full():
                self._log(
                    "  ! cargo full during recycle phase — "
                    "dropping materials at workbench before next pickup"
                )
                self._wb_buffer += self._node.arm_unload_all_materials()
                self._soft(self._node.call_post_process(), "call_post_process (overflow)")

        # Collect whatever disassembly is still pending after the last pickup.
        # The post_process call right after starting that disassembly (above)
        # backs the AMR away from the workbench and rotates it for the exit
        # maneuver, so — unlike the collect call inside the loop, which always
        # re-docks from a distant approach (customer → workbench) that Nav2
        # handles as a normal open-space path — this tail call needs its own
        # re-dock, and specifically needs to go back out to the sub_goal
        # waypoint first. Re-docking straight from the post_process pose
        # (backed up and rotated ~150°, right against the workbench) is a
        # close-quarters, bad-heading replan that Nav2 can't reliably recover
        # from; routing back through the sub_goal restores the same clean
        # open-space approach every other re-dock in this loop gets for free.
        # It also happens to be exactly where we want to park anyway: there's
        # no next pickup to drive off to for this last product, so the AMR
        # would otherwise sit docked at the goal for the whole disassembly
        # wait — parking at the sub_goal instead keeps a safe distance from
        # the workbench for that wait.
        if pending_wb_handle is not None:
            self._wait_at_subgoal_for_recycle(
                pending_wb_handle, pending_wb_pid, workbench_id
            )

    def _collect_recycled_materials(
        self, handle, pid: int, workbench_id: int
    ) -> None:
        """Wait for a RECYCLE WbTask to finish, then pick up every material
        block it produced from the workbench shelf.

        Assumes the AMR is already docked at the workbench goal — used right
        before placing the *next* product on the shelf, where docking is
        unavoidable anyway (see the loop in _run_recycle_phase), so there's
        no safe-distance benefit to retreating first.
        """
        if not self._node.wait_for_wb_task(handle):
            raise ExecutionFailure(f"RECYCLE failed: product={pid}")
        self._pick_recycled_materials(pid, workbench_id)

    def _wait_at_subgoal_for_recycle(
        self, handle, pid: int, workbench_id: int
    ) -> None:
        """Wait for a RECYCLE WbTask to finish while parked at the sub_goal
        (open-space) position — keeps a safe distance from the workbench for
        the whole disassembly instead of idling docked at the goal — then
        dock at the goal to collect the disassembled materials.
        """
        self._require(
            self._node.navigate_subgoal(workbench_id), f"navigate_subgoal({workbench_id})"
        )
        if not self._node.wait_for_wb_task(handle):
            raise ExecutionFailure(f"RECYCLE failed: product={pid}")
        self._require(
            self._node.navigate_goal(workbench_id), f"navigate_goal({workbench_id})"
        )
        self._pick_recycled_materials(pid, workbench_id)

    def _pick_recycled_materials(self, pid: int, workbench_id: int) -> None:
        """Pick up every material block a finished RECYCLE produced from the
        workbench shelf. Caller must already be docked at the workbench goal."""
        for mat_id, cnt in get_material_count(pid).items():
            for _ in range(cnt):
                self._log(f"    ← pick recycled mat {mat_id} from workbench")
                self._require(
                    self._node.arm_pick_material(station_id=workbench_id, material_id=mat_id),
                    f"arm_pick_material(station={workbench_id}, material={mat_id})",
                )
                self._start_ready_intransit_assembly()

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

        self._log(f"Returning surplus recycled materials: {surplus}")
        self._return_materials_to_storage(surplus)

    def _return_materials_to_storage(self, materials: Dict[int, int]) -> None:
        """Return the given {material_id: count} to their home storage stations,
        grouping by destination so each station is visited once."""
        by_station: Dict[int, List[int]] = {}
        for mat_id, cnt in materials.items():
            if cnt <= 0:
                continue
            station_id = self._plan.material_home_station.get(
                mat_id, self._plan.workbench_station_id
            )
            by_station.setdefault(station_id, []).extend([mat_id] * cnt)

        for station_id, mat_ids in by_station.items():
            self._log(f"  → navigate to storage {station_id} to return {mat_ids}")
            self._approach(station_id)
            for mat_id in mat_ids:
                self._require(
                    self._node.arm_return_material_to_storage(
                        material_id=mat_id, station_id=station_id,
                    ),
                    f"arm_return_material_to_storage(material={mat_id}, station={station_id})",
                )
            self._soft(self._node.call_post_process(), "call_post_process (return storage)")

    # ------------------------------------------------------------------
    # Reclaim materials buffered at the workbench (overflow drops)
    # ------------------------------------------------------------------

    def _reclaim_workbench_buffer(self) -> None:
        """Pick back up anything previously dropped at the workbench as an
        overflow buffer (see _wb_buffer). Since Mod 3 removed workbench
        PRODUCE handling, nothing there ever consumes these materials on its
        own — without this step they would be permanently lost from the plan
        even though net_aidlist still needs them (Known Issues #4).

        If cargo fills up again mid-reclaim, the remainder stays in
        self._wb_buffer for the next checkpoint instead of being dropped.
        """
        if not self._wb_buffer:
            return

        pending = {m: c for m, c in self._wb_buffer.items() if c > 0}
        if not pending:
            self._wb_buffer.clear()
            return

        wid = self._plan.workbench_station_id
        self._log(f"Reclaiming materials buffered at workbench: {dict(pending)}")
        self._approach(wid)

        for mat_id, cnt in list(pending.items()):
            for _ in range(cnt):
                if self._node.cargo_is_full():
                    self._log(
                        "  ! cargo full while reclaiming workbench buffer — "
                        "leaving remainder buffered for next checkpoint"
                    )
                    self._soft(self._node.call_post_process(), "call_post_process (reclaim)")
                    return
                self._require(
                    self._node.arm_pick_material(station_id=wid, material_id=mat_id),
                    f"arm_pick_material(station={wid}, material={mat_id})",
                )
                self._wb_buffer[mat_id] -= 1
                if self._wb_buffer[mat_id] <= 0:
                    del self._wb_buffer[mat_id]
                self._start_ready_intransit_assembly()

        self._soft(self._node.call_post_process(), "call_post_process (reclaim)")

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

            self._approach(sid)

            needs_revisit = False
            for mat_id in entry.get('pickup_materials', []):
                if self._node.cargo_is_full():
                    self._log("  ! cargo 2-6 full — overflow drop at workbench")
                    self._soft(self._node.call_post_process(), "call_post_process (pre-overflow)")
                    self._overflow_drop_at_workbench()
                    # Must revisit the station after returning from workbench.
                    needs_revisit = True

                if needs_revisit:
                    self._approach(sid)
                    needs_revisit = False

                self._log(f"  → pick mat {mat_id}")
                self._require(
                    self._node.arm_pick_material(station_id=sid, material_id=mat_id),
                    f"arm_pick_material(station={sid}, material={mat_id})",
                )
                self._start_ready_intransit_assembly()

            self._soft(self._node.call_post_process(), "call_post_process (storage)")

            if self._has_ready_deliveries():
                self._deliver_all()

    # ------------------------------------------------------------------
    # In-transit assembly
    # ------------------------------------------------------------------

    def _approach(self, station_id: int) -> None:
        """Approach station_id following the two-phase navigation rule.

        Always route through the sub_goal (open-space) leg before the
        precision-parking goal leg, and drain any in-transit ASSEMBLE so the
        arm is guaranteed idle before docking. This is the only sanctioned
        way to reach a station's docking goal — calling navigate_goal()
        directly skips the transit window for in-transit assembly and risks
        a close-quarters, bad-heading redock (e.g. from a post_process exit
        pose), which is how the recycle-phase AMR mispositioning bug arose.
        """
        self._require(self._node.navigate_subgoal(station_id), f"navigate_subgoal({station_id})")
        self._wait_for_intransit_assembly()
        self._require(self._node.navigate_goal(station_id), f"navigate_goal({station_id})")

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
        """Navigate to workbench and unload all cargo 2-6 materials as a
        buffer. The unloaded materials are tracked in self._wb_buffer and
        reclaimed at the next checkpoint (see _reclaim_workbench_buffer) —
        they are not simply abandoned there (Known Issues #4)."""
        if self._en_route_to_wb:
            return
        self._en_route_to_wb = True

        wid = self._plan.workbench_station_id
        self._log(f"  → overflow drop: navigate to workbench {wid}")
        self._approach(wid)
        self._wb_buffer += self._node.arm_unload_all_materials()
        self._soft(self._node.call_post_process(), "call_post_process (overflow)")

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
        self._approach(cid)

        # Deliver each completed in-transit slot directly from cargo 7/8.
        for slot in list(self._allocator.get_completed_slots()):
            self._log(
                f"  → deliver product {slot.product_id} from cargo {slot.cargo_id}"
            )
            self._require(
                self._node.arm_deliver(
                    product_id=slot.product_id, from_cargo_id=slot.cargo_id,
                ),
                f"arm_deliver(product={slot.product_id}, cargo={slot.cargo_id})",
            )
            self._started_intransit.discard(slot.cargo_id)
            newly_queued = self._allocator.free_slot(slot.cargo_id)
            if newly_queued is not None:
                self._log(
                    f"  [QUEUE] product {newly_queued} now allocated to cargo {slot.cargo_id}"
                )
                # Materials for newly_queued may already be loaded — check immediately.
                self._start_ready_intransit_assembly()

        self._soft(self._node.call_post_process(), "call_post_process (delivery)")

    # ------------------------------------------------------------------
    # Deferred recycle — produce-then-recycle products with no initial
    # customer-counter stock (run after delivery, before returning home)
    # ------------------------------------------------------------------

    def _run_deferred_recycle_phase(self) -> None:
        """Recycle products that had to be assembled and delivered first.

        These share a product id with a produce order, but had no matching
        stock on the customer counter at plan time — so there was nothing
        to disassemble in Phase 1. Now that the freshly-assembled product
        has been delivered to the customer, pick it back up, disassemble it
        at the workbench, and return every reclaimed material directly to
        its home storage station (production is already done, so nothing
        else needs these materials).
        """
        deferred_ids = self._plan.deferred_recycle_ids
        if not deferred_ids:
            return

        self._log(f"Deferred recycle phase: {len(deferred_ids)} product(s)")
        customer_id = self._plan.customer_station_id
        workbench_id = self._plan.workbench_station_id

        for pid in deferred_ids:
            self._log(f"  → navigate to customer station {customer_id} to reclaim product {pid}")
            self._approach(customer_id)
            self._require(
                self._node.arm_pick_product(station_id=customer_id, product_id=pid),
                f"arm_pick_product(station={customer_id}, product={pid})",
            )
            self._soft(self._node.call_post_process(), "call_post_process (deferred pickup)")

            self._log(f"  → navigate to workbench {workbench_id} for RECYCLE")
            self._approach(workbench_id)
            self._require(
                self._node.arm_unload_product_to_workbench(
                    product_id=pid, station_id=workbench_id,
                ),
                f"arm_unload_product_to_workbench(product={pid}, station={workbench_id})",
            )
            handle = self._node.wb_task_async('RECYCLE', pid)
            # Nothing else to fetch while this disassembles (deferred recycle
            # runs one product at a time) — retreat to the sub_goal to wait
            # instead of idling docked at the goal, same as the Phase 1 tail case.
            self._wait_at_subgoal_for_recycle(handle, pid, workbench_id)
            self._soft(self._node.call_post_process(), "call_post_process (deferred recycle)")

            self._return_materials_to_storage(dict(get_material_count(pid)))

    # ------------------------------------------------------------------
    # Return to home
    # ------------------------------------------------------------------

    def _return_to_home(self) -> None:
        """Navigate back to the home station (0 for A-side, 14 for B-side)."""
        home_id = self._plan.home_station_id
        self._log(f"Returning to home station {home_id}")
        self._approach(home_id)
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

    def _require(self, ok: bool, desc: str) -> None:
        """Raise ExecutionFailure if a required call did not succeed, instead
        of silently continuing with a plan that no longer matches physical
        reality (see Known Issues: none of navigate()/arm_*() results used to
        be checked at all)."""
        if not ok:
            raise ExecutionFailure(f"{desc} failed")

    def _soft(self, ok: bool, desc: str) -> None:
        """Log (but don't abort on) failure of a best-effort call, e.g. the
        post-process exit maneuver — its own failure shouldn't stop the
        mission, but should be visible rather than silently swallowed."""
        if not ok:
            self._node.get_logger().warning(f"[Executor] {desc} did not succeed (continuing)")

    def _log(self, msg: str) -> None:
        self._node.get_logger().info(f"[Executor] {msg}")
