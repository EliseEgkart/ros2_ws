"""A/B arena station id conversion helpers for Plan D."""

SIDE_A_START_GOAL = 0
SIDE_B_START_GOAL = 14

# New ID mapping (non-offset; lookup table required):
#   A side: 0(goal), 1(storage), 2(storage), 3(hybrid), 4(workbench), 6(customer), 7(shared storage)
#   B side: 14(goal), 12(storage), 13(storage), 11(hybrid), 10(workbench), 8(customer), 7(shared storage)

_B_TO_A: dict = {
    14: 0,   # goal
    12: 1,   # storage_1
    13: 2,   # storage_2
    11: 3,   # hybrid_1
    10: 4,   # workbench_1
    8:  6,   # customer_1
    7:  7,   # shared_storage
}
_A_TO_B: dict = {v: k for k, v in _B_TO_A.items()}


def normalize_side(side):
    value = str(side or 'a').strip().lower()
    if value in ('a', 'side_a', 'left'):
        return 'a'
    if value in ('b', 'side_b', 'right'):
        return 'b'
    return 'a'


def side_to_fixed_workbench_station(side):
    """Return the real AMR station id of the fixed workbench for the given side."""
    side = normalize_side(side)
    return 10 if side == 'b' else 4


def side_to_start_goal_station(side):
    """Return the real start/goal station id for the selected arena side."""
    return SIDE_B_START_GOAL if normalize_side(side) == 'b' else SIDE_A_START_GOAL


def amr_station_to_planner_station(station_id, side):
    """Convert real AMR station id to the canonical A-side equivalent.

    A side IDs are returned unchanged.
    B side IDs are mapped via lookup table to the corresponding A side ID.
    Unknown B side IDs are returned unchanged.
    """
    station_id = int(station_id)
    if normalize_side(side) == 'b':
        return _B_TO_A.get(station_id, station_id)
    return station_id


def planner_station_to_amr_station(station_id, side):
    """Convert canonical A-side station id to the real AMR station id for the given side."""
    station_id = int(station_id)
    if normalize_side(side) == 'b':
        return _A_TO_B.get(station_id, station_id)
    return station_id


def nav_target_for_station(station_id, side=None):
    """Return the numeric station id used by NavTask."""
    return int(station_id)
