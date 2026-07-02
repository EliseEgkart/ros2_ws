"""
Loads robocup_waypoint.yaml (the same file the real planner's
DistanceCalculator and the real navigator use) and provides a best-effort
similarity transform from real-world meters to the schematic canvas space
defined in layout_schema.py.

The waypoint file is a work-in-progress on the navigator team's side (as of
this writing it only defines station_0..8_goal, using the "raw" per-side
sequential numbering rather than the real arena_side_utils.py scheme) — this
module is deliberately defensive about that: it only needs TWO reliable
anchor points (canonical station 0 = start/goal, canonical station 6 =
customer_1) to compute a 2D similarity transform (rotation + uniform scale +
translation) via complex-number arithmetic. If those anchors aren't present
yet, transform() returns None and callers should hide/gray out the live AMR
marker rather than guess at a placement.
"""

from __future__ import annotations

import cmath
from typing import Dict, Optional, Tuple

import yaml


class WaypointTransform:
    """Real-meters -> schematic-canvas similarity transform."""

    def __init__(self, real_anchor0: complex, real_anchor1: complex,
                 canvas_anchor0: complex, canvas_anchor1: complex):
        real_delta = real_anchor1 - real_anchor0
        if abs(real_delta) < 1e-9:
            raise ValueError('degenerate anchor pair (identical real positions)')
        self._factor = (canvas_anchor1 - canvas_anchor0) / real_delta
        self._real_anchor0 = real_anchor0
        self._canvas_anchor0 = canvas_anchor0

    def apply(self, x: float, y: float) -> Tuple[float, float]:
        p = self._canvas_anchor0 + self._factor * (complex(x, y) - self._real_anchor0)
        return (p.real, p.imag)


def load_station_goal_positions(waypoint_yaml_path: str) -> Dict[int, Tuple[float, float]]:
    """Return {station_id: (x, y)} for every station_N_goal found (0-20)."""
    if not waypoint_yaml_path:
        return {}
    try:
        with open(waypoint_yaml_path, 'r', encoding='utf-8') as f:
            data = yaml.safe_load(f) or {}
    except Exception:
        return {}

    waypoints = data.get('waypoints', {}) or {}
    positions: Dict[int, Tuple[float, float]] = {}
    for i in range(0, 21):
        wp = waypoints.get(f'station_{i}_goal')
        if wp:
            try:
                positions[i] = (float(wp['position']['x']), float(wp['position']['y']))
            except (KeyError, TypeError, ValueError):
                continue
    return positions


def build_transform(
    real_positions: Dict[int, Tuple[float, float]],
    canvas_anchor0: Tuple[float, float],
    canvas_anchor1: Tuple[float, float],
    anchor_ids: Tuple[int, int] = (0, 6),
) -> Optional[WaypointTransform]:
    """Build a real->canvas transform from two canonical station ids.

    anchor_ids default to (0, 6): canonical start/goal and customer_1 — the
    two roles arena_side_utils.py guarantees exist for either side, and
    which this codebase's other coordinate files (robocup_waypoint.yaml,
    the legacy station_coordinates_a_zone.json) have consistently defined.
    Returns None if either anchor is missing (e.g. an early/partial
    waypoint file) — callers must treat that as "no live position telemetry
    available yet", not fall back to a guess.
    """
    a0, a1 = anchor_ids
    if a0 not in real_positions or a1 not in real_positions:
        return None
    try:
        return WaypointTransform(
            complex(*real_positions[a0]), complex(*real_positions[a1]),
            complex(*canvas_anchor0), complex(*canvas_anchor1),
        )
    except ValueError:
        return None
