"""
Schematic arena layout, proportioned to match map.jpg ("SML Example Layout -
World Cup 2026"): Side A on the left, Side B on the right, a single shared
storage column (07) between them, a customer row along the top, two
start/goal boxes at bottom-center, and "Wait A" / "Wait B" corners.

This is deliberately independent of any particular numeric station_id scheme
(the different eai_task_server task generators disagree with each other and
with the real arena_side_utils.py mapping — see algorithm_description.md's
"Layout inconsistency" note). Station placement is keyed by ROLE, parsed
from the station NAME, which every generator agrees on
(side_a_storage_1, side_a_workbench_2, shared_storage_1, ...). Roles are
grouped and laid out count-adaptively, so a task with 2 or 3 storage
stations per side both render sensibly.

Coordinate space is a fixed logical canvas of CANVAS_W x CANVAS_H; the Tk
view scales this to whatever window size is actually available.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Dict, List

CANVAS_W = 1000.0
CANVAS_H = 580.0

# Horizontal bands (x0, x1)
SIDE_A_X = (50.0, 430.0)
SHARED_X = (455.0, 545.0)
SIDE_B_X = (570.0, 950.0)

# Vertical bands (y0, y1), top to bottom
CUSTOMER_Y = (60.0, 130.0)
WORKBENCH_Y = (150.0, 235.0)
HYBRID_Y = (250.0, 310.0)
STORAGE_Y = (325.0, 470.0)
START_GOAL_Y = (480.0, 535.0)
WAIT_LABEL_Y = 555.0

SHARED_Y = (WORKBENCH_Y[0], STORAGE_Y[1])

STATION_COLORS = {
    'customer': '#f59e0b',
    'workbench': '#a855f7',
    'hybrid': '#c084fc',
    'storage': '#22c55e',
    'shared_storage': '#10b981',
}

ROLE_LABELS = {
    'customer': 'CC',
    'workbench': 'WB',
    'hybrid': 'HWS',
    'storage': 'S',
    'shared_storage': 'S',
}


@dataclass
class StationSlot:
    """One schematic station box, independent of numeric station_id."""
    role: str                  # customer | workbench | hybrid | storage | shared_storage
    side: str                  # side_a | side_b | shared
    index: int                 # 1-based within (side, role)
    name_hint: str             # expected/likely station name, e.g. "side_a_workbench_1"
    x: float
    y: float
    w: float = 60.0
    h: float = 34.0


def _evenly_spaced(x0: float, x1: float, count: int) -> List[float]:
    if count <= 0:
        return []
    if count == 1:
        return [(x0 + x1) / 2.0]
    step = (x1 - x0) / (count - 1)
    return [x0 + step * i for i in range(count)]


def _evenly_spaced_v(y0: float, y1: float, count: int) -> List[float]:
    if count <= 0:
        return []
    if count == 1:
        return [(y0 + y1) / 2.0]
    step = (y1 - y0) / (count - 1)
    return [y0 + step * i for i in range(count)]


def build_side_slots(side: str, counts: Dict[str, int]) -> List[StationSlot]:
    """Build schematic slots for one side ('side_a' or 'side_b').

    counts: {'customer': n, 'workbench': n, 'hybrid': n, 'storage': n}
    Any role with count 0 is skipped. Defaults (when a live Task hasn't been
    seen yet) should pass the map.jpg reference counts: customer=1,
    workbench=2, hybrid=1, storage=2.
    """
    x0, x1 = SIDE_A_X if side == 'side_a' else SIDE_B_X
    slots: List[StationSlot] = []

    for cx, idx in zip(_evenly_spaced(x0, x1, counts.get('customer', 0)),
                        range(1, counts.get('customer', 0) + 1)):
        slots.append(StationSlot('customer', side, idx,
                                  f'{side}_customer_{idx}', cx,
                                  (CUSTOMER_Y[0] + CUSTOMER_Y[1]) / 2.0))

    for cx, idx in zip(_evenly_spaced(x0, x1, counts.get('workbench', 0)),
                        range(1, counts.get('workbench', 0) + 1)):
        slots.append(StationSlot('workbench', side, idx,
                                  f'{side}_workbench_{idx}', cx,
                                  (WORKBENCH_Y[0] + WORKBENCH_Y[1]) / 2.0))

    for cx, idx in zip(_evenly_spaced(x0, x1, counts.get('hybrid', 0)),
                        range(1, counts.get('hybrid', 0) + 1)):
        slots.append(StationSlot('hybrid', side, idx,
                                  f'{side}_hybrid_{idx}', cx,
                                  (HYBRID_Y[0] + HYBRID_Y[1]) / 2.0))

    storage_count = counts.get('storage', 0)
    storage_x = x0 + (x1 - x0) * (0.28 if side == 'side_a' else 0.72)
    for cy, idx in zip(_evenly_spaced_v(STORAGE_Y[0], STORAGE_Y[1], storage_count),
                        range(1, storage_count + 1)):
        slots.append(StationSlot('storage', side, idx,
                                  f'{side}_storage_{idx}', storage_x, cy))

    return slots


def build_shared_slot(count: int = 1) -> List[StationSlot]:
    cx = (SHARED_X[0] + SHARED_X[1]) / 2.0
    slots = []
    for cy, idx in zip(_evenly_spaced_v(SHARED_Y[0], SHARED_Y[1], count), range(1, count + 1)):
        slots.append(StationSlot('shared_storage', 'shared', idx,
                                  'shared_storage_1' if idx == 1 else f'shared_storage_{idx}',
                                  cx, cy, w=70.0, h=(SHARED_Y[1] - SHARED_Y[0]) * 0.9 / max(count, 1)))
    return slots


DEFAULT_COUNTS = {'customer': 1, 'workbench': 2, 'hybrid': 1, 'storage': 2}


def default_slots() -> List[StationSlot]:
    """Full default schematic (used before any Task has been received)."""
    return (
        build_side_slots('side_a', DEFAULT_COUNTS)
        + build_side_slots('side_b', DEFAULT_COUNTS)
        + build_shared_slot(1)
    )


# Decorative (non-data-driven) zones matching map.jpg
START_GOAL_BOXES = [
    {'label': 'START / GOAL', 'x': (SHARED_X[0] + SHARED_X[1]) / 2.0 - 90.0,
     'y': (START_GOAL_Y[0] + START_GOAL_Y[1]) / 2.0, 'w': 110.0, 'h': START_GOAL_Y[1] - START_GOAL_Y[0]},
    {'label': 'START / GOAL', 'x': (SHARED_X[0] + SHARED_X[1]) / 2.0 + 90.0,
     'y': (START_GOAL_Y[0] + START_GOAL_Y[1]) / 2.0, 'w': 110.0, 'h': START_GOAL_Y[1] - START_GOAL_Y[0]},
]

WAIT_LABELS = [
    {'label': 'Wait A', 'x': (SIDE_A_X[0] + SIDE_A_X[1]) / 2.0, 'y': WAIT_LABEL_Y},
    {'label': 'Robot Fleets', 'x': CANVAS_W / 2.0, 'y': WAIT_LABEL_Y},
    {'label': 'Wait B', 'x': (SIDE_B_X[0] + SIDE_B_X[1]) / 2.0, 'y': WAIT_LABEL_Y},
]

SIDE_TITLE_Y = 25.0
