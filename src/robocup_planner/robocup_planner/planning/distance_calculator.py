"""
Distance calculator using the AMR team's waypoint YAML.

Bezier curve path model:
  Each station approach has two legs:
    1. Cruise leg: previous_goal → this_sub_goal
       Modelled as a quadratic Bezier curve whose tangent at sub_goal is
       aligned with the sub_goal→goal approach direction. This captures the
       curved path the AMR takes as it aligns itself for precision parking.
    2. Dock leg: sub_goal → goal  (straight line, precision parking)

  Total path distance = Bezier arc length (cruise) + Euclidean (dock).

  When sub_goal data is unavailable for a station, falls back to Euclidean
  straight-line distance between goal positions.

Control point placement:
  For a quadratic Bezier B(t) = (1-t)²P0 + 2(1-t)t·P1 + t²·P2
  with P0=from_goal, P2=sub_goal, the tangent at P2 is 2(P2-P1).
  To be tangent to (sub_goal→goal) at P2, we place P1 on the line through
  sub_goal pointing OPPOSITE to (goal-sub_goal), at distance = |sub_goal-goal|.

The YAML will be updated by the AMR team on competition day, so all distances
are recalculated at runtime by loading the file fresh.
"""

import math
import yaml
from typing import Dict, Optional, Tuple


class DistanceCalculator:
    def __init__(self, waypoint_yaml_path: str):
        with open(waypoint_yaml_path, 'r') as f:
            data = yaml.safe_load(f)
        self._waypoints: dict = data['waypoints']
        self._goal_positions: Dict[int, Tuple[float, float]] = {}
        self._subgoal_positions: Dict[int, Tuple[float, float]] = {}
        self._parse_positions()

        # Backward-compatible alias
        self._positions = self._goal_positions

    def _parse_positions(self) -> None:
        for i in range(0, 21):
            key_goal = f'station_{i}_goal'
            key_sub = f'station_{i}_sub_goal'
            wp_goal = self._waypoints.get(key_goal)
            if wp_goal and wp_goal.get('position'):
                self._goal_positions[i] = (
                    wp_goal['position']['x'],
                    wp_goal['position']['y'],
                )
            wp_sub = self._waypoints.get(key_sub)
            if wp_sub and wp_sub.get('position'):
                self._subgoal_positions[i] = (
                    wp_sub['position']['x'],
                    wp_sub['position']['y'],
                )

    # ------------------------------------------------------------------
    # Low-level accessors
    # ------------------------------------------------------------------

    def get_position(self, station_id: int) -> Optional[Tuple[float, float]]:
        """Return the goal (docking) position for station_id."""
        return self._goal_positions.get(station_id)

    def get_subgoal_position(self, station_id: int) -> Optional[Tuple[float, float]]:
        """Return the sub_goal (approach) position for station_id."""
        return self._subgoal_positions.get(station_id)

    # ------------------------------------------------------------------
    # Bezier helper
    # ------------------------------------------------------------------

    @staticmethod
    def _bezier_arc_length(
        p0: Tuple[float, float],
        p1: Tuple[float, float],
        p2: Tuple[float, float],
        n: int = 24,
    ) -> float:
        """Approximate arc length of a quadratic Bezier curve via line segments.

        P0 = start, P1 = control point, P2 = end.
        n controls approximation quality; 24 segments is accurate to ~0.1 % for
        typical arena distances.
        """
        length = 0.0
        prev = p0
        for i in range(1, n + 1):
            t = i / n
            mt = 1.0 - t
            x = mt * mt * p0[0] + 2.0 * mt * t * p1[0] + t * t * p2[0]
            y = mt * mt * p0[1] + 2.0 * mt * t * p1[1] + t * t * p2[1]
            length += math.sqrt((x - prev[0]) ** 2 + (y - prev[1]) ** 2)
            prev = (x, y)
        return length

    def _bezier_path_to_station(
        self,
        from_pos: Tuple[float, float],
        to_id: int,
    ) -> float:
        """Full Bezier path distance from from_pos to to_id.

        Cruise leg: from_pos → sub_goal  (quadratic Bezier, tangent to approach)
        Dock leg:   sub_goal → goal      (straight line)
        Falls back to Euclidean if sub_goal is missing.
        """
        p_goal = self._goal_positions.get(to_id)
        if p_goal is None:
            return float('inf')

        p_sub = self._subgoal_positions.get(to_id)
        if p_sub is None:
            # No sub_goal data — plain Euclidean to goal
            return math.sqrt(
                (from_pos[0] - p_goal[0]) ** 2 + (from_pos[1] - p_goal[1]) ** 2
            )

        # Dock leg (straight line)
        dx_dock = p_goal[0] - p_sub[0]
        dy_dock = p_goal[1] - p_sub[1]
        dock_len = math.sqrt(dx_dock ** 2 + dy_dock ** 2)

        if dock_len < 1e-6:
            # sub_goal == goal; treat as straight Euclidean
            return math.sqrt(
                (from_pos[0] - p_goal[0]) ** 2 + (from_pos[1] - p_goal[1]) ** 2
            )

        # Control point: on the line through sub_goal pointing OPPOSITE to approach dir,
        # at the same distance as dock_len so the curve is roughly symmetric.
        p1 = (
            p_sub[0] - dx_dock,
            p_sub[1] - dy_dock,
        )
        cruise_len = self._bezier_arc_length(from_pos, p1, p_sub)
        return cruise_len + dock_len

    # ------------------------------------------------------------------
    # Public distance API
    # ------------------------------------------------------------------

    def station_to_station(self, from_id: int, to_id: int) -> float:
        """Bezier path distance from from_id goal to to_id (cruise + dock)."""
        from_pos = self._goal_positions.get(from_id)
        if from_pos is None:
            return float('inf')
        return self._bezier_path_to_station(from_pos, to_id)

    def point_to_station(self, x: float, y: float, station_id: int) -> float:
        """Bezier path distance from an arbitrary (x, y) to station_id."""
        return self._bezier_path_to_station((x, y), station_id)

    def station_to_station_euclidean(self, from_id: int, to_id: int) -> float:
        """Straight-line (Euclidean) fallback between two station goal positions."""
        a = self._goal_positions.get(from_id)
        b = self._goal_positions.get(to_id)
        if a is None or b is None:
            return float('inf')
        return math.sqrt((a[0] - b[0]) ** 2 + (a[1] - b[1]) ** 2)

    def estimate_travel_time(
        self,
        from_id: int,
        to_id: int,
        driving_velocity: float,
        parking_duration: float,
        exiting_duration: float,
    ) -> float:
        """Travel time estimate using Bezier path distance.

        Uses the full cruise+dock path length divided by driving_velocity,
        then adds fixed parking and exiting overhead.
        """
        dist = self.station_to_station(from_id, to_id)
        if driving_velocity <= 0:
            return float('inf')
        return dist / driving_velocity + parking_duration + exiting_duration
