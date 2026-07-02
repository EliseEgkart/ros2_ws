"""
Visual constants shared by canvas_view.py.

Station CANVAS POSITIONS are no longer computed here — as of 2026-07-02
they're derived directly from real robocup_waypoint.yaml positions (see
waypoints.py) so the rendered station boxes and the live AMR marker are
guaranteed to line up, instead of being two independently-guessed
coordinate systems. This module keeps only role colors/labels and the
canvas geometry constants (overall size, margins, the "unresolved
stations" tray for anything the waypoint file doesn't have a position for
yet).
"""

from __future__ import annotations

CANVAS_W = 1000.0
CANVAS_H = 580.0

# Region real station positions are fit into (fit_transform in waypoints.py
# preserves aspect ratio and centers within this box).
ARENA_MARGIN = 70.0
ARENA_X0 = ARENA_MARGIN
ARENA_Y0 = ARENA_MARGIN
ARENA_X1 = CANVAS_W - ARENA_MARGIN
ARENA_Y1 = CANVAS_H - 90.0  # leave room for the unresolved-station tray

# Stations with no known real position (waypoint_yaml doesn't have their
# station_id yet) are listed here instead of silently vanishing.
TRAY_Y = CANVAS_H - 45.0

STATION_BOX_W = 64.0
STATION_BOX_H = 34.0

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

ZONE_TINTS = {
    'side_a': '#14213d',
    'side_b': '#3a1530',
}
