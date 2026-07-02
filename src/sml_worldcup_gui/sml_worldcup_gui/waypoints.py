"""
Loads robocup_waypoint.yaml (the same file the real planner's
DistanceCalculator and the real navigator use) and provides a single,
shared real-meters -> canvas-pixels transform.

Design note (2026-07-02, after real-hardware testing showed station boxes
and the AMR marker landing in different places): the first version of this
module built the canvas layout from a hand-drawn schematic grid, and
computed a *separate* 2-anchor similarity transform only for the AMR
marker. Those two coordinate systems were never guaranteed to agree except
at the two anchor points — everywhere else, misalignment was the expected
outcome, not a bug in the data. This version removes the hand-drawn grid
entirely: every station's canvas position AND the AMR marker are derived
from the same real waypoint positions through the same fit_transform(),
so they are aligned by construction.

resolve_station_xy() mirrors mock_nav_node.py's _station_coord() resolution
order exactly (try the station's own raw station_id first, then the
B-side -> A-side canonical id) so that a station renders at the same real
position the AMR actually navigates to for it — not a different one.

The waypoint file is a work-in-progress on the navigator team's side (as of
this writing it only defines station_0..8_goal). Any station whose id has
no entry yet resolves to None; callers must show it as
"position unknown" rather than guess.
"""

from __future__ import annotations

from typing import Dict, List, Optional, Sequence, Tuple

import yaml

# Mirrors sml_system_pkg/arena_side_utils.py's _B_TO_A table. Duplicated
# (not imported) so this GUI package doesn't need a dependency on
# sml_system_pkg for seven integers — same decoupling precedent
# side_panel.py already uses for its product-name table.
_B_TO_A = {14: 0, 12: 1, 13: 2, 11: 3, 10: 4, 8: 6, 72: 71}


def load_station_goal_positions(waypoint_yaml_path: str) -> Dict[int, Tuple[float, float]]:
    """Return {station_id: (x, y)} for every station_N_goal found (0-99)."""
    if not waypoint_yaml_path:
        return {}
    try:
        with open(waypoint_yaml_path, 'r', encoding='utf-8') as f:
            data = yaml.safe_load(f) or {}
    except Exception:
        return {}

    waypoints = data.get('waypoints', {}) or {}
    positions: Dict[int, Tuple[float, float]] = {}
    for i in range(0, 100):
        wp = waypoints.get(f'station_{i}_goal')
        if wp:
            try:
                positions[i] = (float(wp['position']['x']), float(wp['position']['y']))
            except (KeyError, TypeError, ValueError):
                continue
    return positions


def resolve_station_xy(
    station_id: int, side: str, real_positions: Dict[int, Tuple[float, float]]
) -> Optional[Tuple[float, float]]:
    """Real (x, y) for a station, or None if the waypoint file doesn't have
    it yet. Tries the raw id first, then the B->A canonical id — the same
    order mock_nav_node._station_coord() uses, so a rendered station sits at
    the exact position the AMR will actually navigate to for it."""
    station_id = int(station_id)
    if station_id in real_positions:
        return real_positions[station_id]
    if side == 'side_b' and station_id in _B_TO_A:
        canonical = _B_TO_A[station_id]
        if canonical in real_positions:
            return real_positions[canonical]
    return None


class FitTransform:
    """Uniform-scale, y-flipped fit of a set of real (x, y) meter points into
    a canvas rectangle, preserving aspect ratio and true relative geometry.

    y is flipped (screen y grows downward; robot/world y direction is
    otherwise unconstrained) so the picture doesn't come out mirrored
    top-to-bottom; there is no verified ground truth tying "up" in the real
    coordinate frame to "up" in map.jpg, so this is a consistent choice, not
    a guess at station roles.
    """

    def __init__(self, real_points: Sequence[Tuple[float, float]],
                 canvas_x0: float, canvas_y0: float, canvas_x1: float, canvas_y1: float):
        xs = [p[0] for p in real_points]
        ys = [p[1] for p in real_points]
        self._min_x, self._max_x = min(xs), max(xs)
        self._min_y, self._max_y = min(ys), max(ys)
        real_w = max(self._max_x - self._min_x, 1e-6)
        real_h = max(self._max_y - self._min_y, 1e-6)

        canvas_w = canvas_x1 - canvas_x0
        canvas_h = canvas_y1 - canvas_y0
        self._scale = min(canvas_w / real_w, canvas_h / real_h)

        # Center the (possibly non-square) real bounding box inside the
        # canvas rectangle.
        used_w = real_w * self._scale
        used_h = real_h * self._scale
        self._off_x = canvas_x0 + (canvas_w - used_w) / 2.0
        self._off_y = canvas_y0 + (canvas_h - used_h) / 2.0

    def apply(self, x: float, y: float) -> Tuple[float, float]:
        cx = self._off_x + (x - self._min_x) * self._scale
        cy = self._off_y + (self._max_y - y) * self._scale  # y-flip
        return (cx, cy)


def fit_transform(
    real_points: Sequence[Tuple[float, float]],
    canvas_x0: float, canvas_y0: float, canvas_x1: float, canvas_y1: float,
) -> Optional[FitTransform]:
    """Build a FitTransform from at least 2 distinct real points. Returns
    None if fewer than 2 points are given (nothing sensible to fit)."""
    if len(real_points) < 2:
        return None
    return FitTransform(real_points, canvas_x0, canvas_y0, canvas_x1, canvas_y1)
