"""
Station role parsing, shared by canvas_view.py and side_panel.py.

Every eai_task_server task generator disagrees on numeric station_id
schemes, but all of them name stations consistently:
  side_a_storage_1, side_a_storage_2, side_a_workbench_1, side_a_workbench_2,
  side_a_hybrid_1, side_a_customer_1, side_b_..., shared_storage_1
(see algorithm_description.md's "Layout inconsistency" note). Parsing by
name instead of station_id is what lets this GUI render any of them
correctly.
"""

from __future__ import annotations

import re
from typing import Dict, List, NamedTuple, Sequence

from sml_messages.msg import Station

_NAME_RE = re.compile(r'^(side_a|side_b)_(storage|workbench|hybrid|customer)_(\d+)$')

_TYPE_TO_ROLE = {
    Station.ST_STORAGE: 'storage',
    Station.ST_WORKBENCH: 'workbench',
    Station.ST_CUSTOMER: 'customer',
    Station.ST_HYBRID: 'hybrid',
}


class RoleInfo(NamedTuple):
    side: str   # 'side_a' | 'side_b' | 'shared'
    role: str   # storage | workbench | hybrid | customer | shared_storage
    index: int  # 1-based within (side, role); best-effort for shared/unnamed


def parse_role(station: Station) -> RoleInfo:
    name = str(getattr(station, 'name', '') or '').strip().lower()

    # Name match takes priority over any numeric-id heuristic: raw station_id
    # 7 means "side_a_customer_1" under task_publisher.py's scheme but
    # "shared_storage_1" under order.py's — the exact inconsistency this
    # name-first design exists to route around (see module docstring).
    m = _NAME_RE.match(name)
    if m:
        side, role, idx = m.group(1), m.group(2), int(m.group(3))
        return RoleInfo(side, role, idx)

    # Only fall back to id/substring heuristics when the name doesn't match
    # the standard side_a|side_b_<role>_<n> pattern at all. 71/72 are
    # unambiguous real shared-storage approach ids (arena_side_utils.py);
    # bare "7" is deliberately excluded since it collides with
    # task_publisher.py's numbering.
    if 'shared' in name or int(station.station_id) in (71, 72):
        return RoleInfo('shared', 'shared_storage', 1)

    # Fallback: infer side from name prefix, role from station_type.
    side = 'side_a' if name.startswith('side_a') else (
        'side_b' if name.startswith('side_b') else 'side_a'
    )
    role = _TYPE_TO_ROLE.get(int(station.station_type), 'storage')
    return RoleInfo(side, role, 1)


def group_by_role(stations: Sequence[Station]) -> Dict[str, List[Station]]:
    """Return {'side_a': {...}, 'side_b': {...}, 'shared': [...]} shaped as
    {side: {role: [Station, ...]}} for side_a/side_b, and {'shared': [...]}."""
    grouped: Dict[str, Dict[str, List[Station]]] = {
        'side_a': {'storage': [], 'workbench': [], 'hybrid': [], 'customer': []},
        'side_b': {'storage': [], 'workbench': [], 'hybrid': [], 'customer': []},
    }
    shared: List[Station] = []

    for st in stations:
        info = parse_role(st)
        if info.side == 'shared':
            shared.append(st)
            continue
        grouped[info.side].setdefault(info.role, []).append(st)

    for side in ('side_a', 'side_b'):
        for role, items in grouped[side].items():
            items.sort(key=lambda s: parse_role(s).index)

    return {'side_a': grouped['side_a'], 'side_b': grouped['side_b'], 'shared': shared}
