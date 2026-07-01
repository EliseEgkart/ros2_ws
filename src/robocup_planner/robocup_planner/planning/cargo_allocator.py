"""
Cargo slot manager for in-transit assembly (cargo IDs 7 and 8).

All produce products are assembled in-transit by the AMR cargo arm.
When more than 2 products are ordered, they are queued; as soon as a slot is
freed after delivery, the next queued product is allocated to that slot.

Priority rule for slot assignment:
  Sort key: (num_blocks / weight, product_id)
    - Higher weight → assigned earlier (lower sort value).
    - Fewer blocks → assigned earlier (simpler assemblies free the slot sooner).
    - Smallest product_id as deterministic tie-break.

Slot lifecycle:
  allocate()             → called once at plan time; fills slots 7/8 and queues the rest.
  mark_slot_assembled()  → called when ASSEMBLE succeeds; marks slot as deliverable.
  free_slot()            → called after delivery; releases the slot and auto-assigns
                            the next queued product (if any).
"""

from typing import Dict, List, Optional

from robocup_planner.product_catalog import (
    get_build_order,
    is_intransit_eligible,
)

INTRANSIT_CARGO_IDS = (7, 8)


class IntransitSlot:
    def __init__(self, cargo_id: int, product_id: int):
        self.cargo_id = cargo_id
        self.product_id = product_id
        self.build_order: List[int] = get_build_order(product_id)
        self.placed: List[int] = []

    @property
    def next_needed(self) -> Optional[int]:
        idx = len(self.placed)
        if idx < len(self.build_order):
            return self.build_order[idx]
        return None

    @property
    def is_complete(self) -> bool:
        return self.placed == self.build_order

    def confirm_block(self, material_id: int) -> bool:
        """Record block as placed. Returns True if assembly is now complete."""
        self.placed.append(material_id)
        return self.is_complete

    def mark_complete(self) -> None:
        """Mark the slot as containing the completed assembled product."""
        self.placed = list(self.build_order)


class CargoAllocator:
    def __init__(self):
        self._slots: Dict[int, Optional[IntransitSlot]] = {
            cargo_id: None for cargo_id in INTRANSIT_CARGO_IDS
        }
        # Products waiting for a free slot, in priority order.
        self._queue: List[int] = []

    def allocate(
        self,
        produce_product_ids: List[int],
        weights: Optional[Dict[int, float]] = None,
    ) -> Dict[int, int]:
        """Assign eligible products to cargo 7/8; queue the rest.

        Sort key: (num_blocks / weight, product_id)
          Higher weight → lower sort value → earlier slot assignment.

        Returns {product_id: cargo_id} for the products immediately assigned
        to a slot.  Products placed in the queue are not in the returned dict
        but will be assigned as slots free up via free_slot().
        """
        if weights is None:
            weights = {}

        eligible = sorted(
            [pid for pid in produce_product_ids if is_intransit_eligible(pid)],
            key=lambda pid: (
                len(get_build_order(pid)) / max(weights.get(pid, 1.0), 1e-9),
                pid,
            ),
        )

        allocation: Dict[int, int] = {}
        queue_buf: List[int] = []

        for pid in eligible:
            assigned = False
            for cargo_id in INTRANSIT_CARGO_IDS:
                if self._slots[cargo_id] is None:
                    self._slots[cargo_id] = IntransitSlot(cargo_id, pid)
                    allocation[pid] = cargo_id
                    assigned = True
                    break
            if not assigned:
                queue_buf.append(pid)

        self._queue = queue_buf
        return allocation

    def _try_assign_from_queue(self) -> Optional[int]:
        """Assign the next queued product to the first free slot, if both exist.
        Returns the newly assigned product_id, or None."""
        if not self._queue:
            return None
        for cargo_id in INTRANSIT_CARGO_IDS:
            if self._slots[cargo_id] is None:
                pid = self._queue.pop(0)
                self._slots[cargo_id] = IntransitSlot(cargo_id, pid)
                return pid
        return None

    # ------------------------------------------------------------------
    # Runtime slot management
    # ------------------------------------------------------------------

    def find_slot_for_block(self, material_id: int) -> Optional[int]:
        """Return cargo_id waiting for material_id as its next block, or None."""
        for cargo_id, slot in self._slots.items():
            if slot is not None and slot.next_needed == material_id:
                return cargo_id
        return None

    def confirm_placed(self, cargo_id: int, material_id: int) -> bool:
        """Record that material_id was placed on cargo_id.
        Returns True if assembly is now complete."""
        slot = self._slots.get(cargo_id)
        if slot is None:
            return False
        return slot.confirm_block(material_id)

    def get_completed_slots(self) -> List[IntransitSlot]:
        """Return slots whose assembly is complete (product ready to deliver)."""
        return [s for s in self._slots.values() if s is not None and s.is_complete]

    def free_slot(self, cargo_id: int) -> Optional[int]:
        """Release cargo_id after delivery. Auto-assigns next queued product.
        Returns the newly assigned product_id, or None if queue is empty."""
        self._slots[cargo_id] = None
        return self._try_assign_from_queue()

    def mark_assembled(self, product_id: int) -> Optional[int]:
        """Mark product_id as assembled on its slot. Returns cargo_id or None."""
        cargo_id = self.get_product_slot(product_id)
        if cargo_id is None:
            return None
        if not self.mark_slot_assembled(cargo_id, product_id):
            return None
        return cargo_id

    def mark_slot_assembled(self, cargo_id: int, product_id: int) -> bool:
        """Mark the specific cargo slot as assembled.

        Product IDs are not unique when an order contains duplicates, so runtime
        completion must be keyed by cargo_id.
        """
        slot = self._slots.get(cargo_id)
        if slot is None or int(slot.product_id) != int(product_id):
            return False
        slot.mark_complete()
        return True

    # ------------------------------------------------------------------
    # Query helpers
    # ------------------------------------------------------------------

    def allocated_products(self) -> List[int]:
        """Products currently occupying a cargo slot (not queued)."""
        return [s.product_id for s in self._slots.values() if s is not None]

    def queued_products(self) -> List[int]:
        """Products waiting for a free cargo slot, in priority order."""
        return list(self._queue)

    def is_cargo_allocated(self, cargo_id: int) -> bool:
        return self._slots.get(cargo_id) is not None

    def get_slot_product(self, cargo_id: int) -> Optional[int]:
        """Return the product_id allocated to cargo_id, or None."""
        slot = self._slots.get(cargo_id)
        return slot.product_id if slot is not None else None

    def get_product_slot(self, product_id: int) -> Optional[int]:
        """Return the cargo_id allocated to product_id (in-slot only), or None."""
        for cargo_id, slot in self._slots.items():
            if slot is not None and slot.product_id == product_id:
                return cargo_id
        return None

    def has_in_progress(self) -> bool:
        """True if any slot has blocks placed but is not yet complete."""
        return any(
            s is not None and len(s.placed) > 0 and not s.is_complete
            for s in self._slots.values()
        )
