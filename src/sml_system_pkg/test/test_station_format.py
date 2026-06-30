from sml_msgs.msg import Station

from sml_system_pkg.arena_side_utils import (
    _A_TO_B,
    _B_TO_A,
    amr_station_to_planner_station,
    planner_station_to_amr_station,
    side_to_fixed_workbench_station,
    side_to_start_goal_station,
)
from sml_system_pkg.order_server import STATION_DEFS


def test_side_station_id_conversions():
    assert side_to_start_goal_station('a') == 0
    assert side_to_start_goal_station('b') == 14
    assert side_to_fixed_workbench_station('a') == 4
    assert side_to_fixed_workbench_station('b') == 10

    # A side: IDs are passed through unchanged
    a_ids = [0, 1, 2, 3, 4, 6, 71]
    for sid in a_ids:
        assert amr_station_to_planner_station(sid, 'a') == sid
        assert planner_station_to_amr_station(sid, 'a') == sid

    # B side: explicit lookup table round-trip
    for b_id, a_id in _B_TO_A.items():
        assert amr_station_to_planner_station(b_id, 'b') == a_id
        assert planner_station_to_amr_station(a_id, 'b') == b_id


def test_station_types_match_arena_format():
    # STATION_DEFS: (suffix, a_id, b_id, station_type)
    a_types = {a_id: station_type for _, a_id, _b_id, station_type in STATION_DEFS}
    b_types = {b_id: station_type for _, _a_id, b_id, station_type in STATION_DEFS}

    assert {sid for sid, stype in a_types.items() if stype == Station.ST_STORAGE} == {1, 2, 71}
    assert {sid for sid, stype in a_types.items() if stype == Station.ST_HYBRID} == {3}
    assert {sid for sid, stype in a_types.items() if stype == Station.ST_WORKBENCH} == {4}
    assert {sid for sid, stype in a_types.items() if stype == Station.ST_CUSTOMER} == {6}

    assert {sid for sid, stype in b_types.items() if stype == Station.ST_STORAGE} == {12, 13, 72}
    assert {sid for sid, stype in b_types.items() if stype == Station.ST_HYBRID} == {11}
    assert {sid for sid, stype in b_types.items() if stype == Station.ST_WORKBENCH} == {10}
    assert {sid for sid, stype in b_types.items() if stype == Station.ST_CUSTOMER} == {8}
