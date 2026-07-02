"""
Optional, user-editable layout overrides (station_layout.yaml / .json).

Station canvas positions and the AMR marker are driven by real
robocup_waypoint.yaml meters by default (see waypoints.py) — that's what
guarantees the AMR marker actually lands on the station it's visiting.
This module adds an *optional* second layer on top: a hand-authored file
that repositions specific, named stations (and adds purely decorative
boxes/labels — start/goal areas, wait zones) to match a reference diagram
such as map.jpg more closely, for presentation purposes.

Design constraint that keeps this from silently reintroducing the exact
station/AMR misalignment bug this GUI was rebuilt to fix: overriding a
station's position here only changes where *that station's box* is drawn.
The AMR marker is always placed via the real-waypoint transform. If a
station most people care about watching live is overridden, its box and
the AMR marker will only coincide when the AMR is actually there in real
life *and* the override's position roughly matches the real geometry —
that tradeoff is the user's explicit choice when they author the file, not
an accident. Stations left out of the file keep the accuracy-guaranteed
real-waypoint placement.

File format (YAML or JSON, keys are the same either way)::

    canvas: {width: 1000, height: 580}       # optional, informational only
    stations:
      side_a_storage_1: {x: 120, y: 480}
      side_a_workbench_1: {x: 90, y: 180}
      shared_storage_1: {x: 500, y: 300}
    boxes:                                    # decorative only, no click target
      - {x0: 20, y0: 500, x1: 160, y1: 560, label: "START / GOAL A", dash: true}
    labels:
      - {x: 500, y: 40, text: "SML Arena — World Cup 2026"}

All keys are optional; a missing/unparsable file yields an empty
LayoutOverrides (i.e. pure real-waypoint mode, today's default behavior).
"""

from __future__ import annotations

import json
import os
from dataclasses import dataclass, field
from typing import Dict, List, Tuple

import yaml


@dataclass
class LayoutOverrides:
    station_positions: Dict[str, Tuple[float, float]] = field(default_factory=dict)
    boxes: List[dict] = field(default_factory=list)
    labels: List[dict] = field(default_factory=list)


def load_layout_overrides(path: str) -> LayoutOverrides:
    if not path or not os.path.isfile(path):
        return LayoutOverrides()
    try:
        with open(path, 'r', encoding='utf-8') as f:
            if path.lower().endswith('.json'):
                data = json.load(f) or {}
            else:
                data = yaml.safe_load(f) or {}
    except Exception:
        return LayoutOverrides()

    stations: Dict[str, Tuple[float, float]] = {}
    for name, xy in (data.get('stations') or {}).items():
        try:
            stations[str(name)] = (float(xy['x']), float(xy['y']))
        except (KeyError, TypeError, ValueError):
            continue

    boxes = [b for b in (data.get('boxes') or []) if isinstance(b, dict)]
    labels = [l for l in (data.get('labels') or []) if isinstance(l, dict)]

    return LayoutOverrides(station_positions=stations, boxes=boxes, labels=labels)
