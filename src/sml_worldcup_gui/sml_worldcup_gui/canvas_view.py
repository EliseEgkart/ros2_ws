"""
tk.Canvas renderer for the arena. Static schematic (zones/labels/start-goal
boxes) always draws immediately, independent of any live data; stations
redraw whenever a new Task arrives, grouped/counted by role (see
station_roles.py) rather than trusting numeric station_id. The AMR marker
draws whenever the ros_bridge supplies a transformed canvas position.
"""

from __future__ import annotations

import tkinter as tk
from typing import Callable, Dict, List, Optional, Tuple

from sml_messages.msg import Station, Task

from . import layout_schema as L
from .station_roles import group_by_role

BACKGROUND = '#0b1220'
ZONE_A = '#14213d'
ZONE_B = '#3a1530'
ZONE_SHARED = '#123028'
TEXT = '#e5edf8'
MUTED = '#94a3b8'
AMR_COLOR = '#38bdf8'
START_GOAL_COLOR = '#ef4444'
GRID_LINE = '#1e293b'


class ArenaCanvas:
    def __init__(self, parent: tk.Widget, on_select: Optional[Callable[[str, str, List[int]], None]] = None):
        self.canvas = tk.Canvas(parent, bg=BACKGROUND, highlightthickness=0)
        self.canvas.pack(fill=tk.BOTH, expand=True)
        self._on_select = on_select

        self.side_filter = 'all'  # 'a' | 'b' | 'all'
        self._slots: List[L.StationSlot] = L.default_slots()
        self._station_by_slot: Dict[Tuple[str, str, int], Station] = {}
        self._amr_xy: Optional[Tuple[float, float]] = None
        self._amr_live = False

        self.canvas.bind('<Configure>', lambda _e: self._redraw())

    # ------------------------------------------------------------------
    # Public API
    # ------------------------------------------------------------------

    def set_side_filter(self, side: str) -> None:
        self.side_filter = side
        self._redraw()

    def set_task(self, task: Task) -> None:
        grouped = group_by_role(list(task.arena_layout))

        counts_a = {role: len(items) for role, items in grouped['side_a'].items()}
        counts_b = {role: len(items) for role, items in grouped['side_b'].items()}
        shared_count = max(1, len(grouped['shared']))

        slots = L.build_side_slots('side_a', counts_a) + L.build_side_slots('side_b', counts_b)
        slots += L.build_shared_slot(shared_count)
        self._slots = slots

        station_by_slot: Dict[Tuple[str, str, int], Station] = {}
        for side_key, role_map in (('side_a', grouped['side_a']), ('side_b', grouped['side_b'])):
            for role, items in role_map.items():
                for i, st in enumerate(items, start=1):
                    station_by_slot[(side_key, role, i)] = st
        for i, st in enumerate(grouped['shared'], start=1):
            station_by_slot[('shared', 'shared_storage', i)] = st
        self._station_by_slot = station_by_slot

        self._redraw()

    def set_amr_position(self, canvas_xy: Optional[Tuple[float, float]], live: bool) -> None:
        self._amr_xy = canvas_xy
        self._amr_live = live
        self._redraw_amr_only()

    # ------------------------------------------------------------------
    # Viewport / coordinate transform
    # ------------------------------------------------------------------

    def _view_bounds(self) -> Tuple[float, float, float, float]:
        if self.side_filter == 'a':
            return (L.SIDE_A_X[0] - 20, 0, L.SHARED_X[1] + 20, L.CANVAS_H)
        if self.side_filter == 'b':
            return (L.SHARED_X[0] - 20, 0, L.SIDE_B_X[1] + 20, L.CANVAS_H)
        return (0, 0, L.CANVAS_W, L.CANVAS_H)

    def _to_widget(self, x: float, y: float) -> Tuple[float, float]:
        x0, y0, x1, y1 = self._view_bounds()
        w = max(1, self.canvas.winfo_width())
        h = max(1, self.canvas.winfo_height())
        scale = min(w / max(x1 - x0, 1e-6), h / max(y1 - y0, 1e-6))
        off_x = (w - (x1 - x0) * scale) / 2.0
        off_y = (h - (y1 - y0) * scale) / 2.0
        return (off_x + (x - x0) * scale, off_y + (y - y0) * scale)

    # ------------------------------------------------------------------
    # Drawing
    # ------------------------------------------------------------------

    def _redraw(self) -> None:
        self.canvas.delete('all')
        self._draw_zones()
        self._draw_start_goal()
        self._draw_wait_labels()
        self._draw_stations()
        self._draw_amr()

    def _redraw_amr_only(self) -> None:
        self.canvas.delete('amr')
        self._draw_amr()

    def _draw_zones(self) -> None:
        for (x0, x1), color, label in (
            (L.SIDE_A_X, ZONE_A, 'Side A'),
            (L.SIDE_B_X, ZONE_B, 'Side B'),
            (L.SHARED_X, ZONE_SHARED, ''),
        ):
            p0 = self._to_widget(x0 - 15, 10)
            p1 = self._to_widget(x1 + 15, L.CANVAS_H - 40)
            self.canvas.create_rectangle(*p0, *p1, fill=color, outline='', tags='zone')
            if label:
                lp = self._to_widget((x0 + x1) / 2.0, L.SIDE_TITLE_Y)
                self.canvas.create_text(*lp, text=label, fill=TEXT, font=('Segoe UI', 13, 'bold'))

    def _draw_start_goal(self) -> None:
        for box in L.START_GOAL_BOXES:
            p0 = self._to_widget(box['x'] - box['w'] / 2, box['y'] - box['h'] / 2)
            p1 = self._to_widget(box['x'] + box['w'] / 2, box['y'] + box['h'] / 2)
            self.canvas.create_rectangle(
                *p0, *p1, outline=START_GOAL_COLOR, dash=(4, 3), width=2,
            )
            cp = self._to_widget(box['x'], box['y'])
            self.canvas.create_text(*cp, text=box['label'], fill=START_GOAL_COLOR, font=('Segoe UI', 8, 'bold'))

    def _draw_wait_labels(self) -> None:
        for item in L.WAIT_LABELS:
            p = self._to_widget(item['x'], item['y'])
            self.canvas.create_text(*p, text=item['label'], fill=MUTED, font=('Segoe UI', 9))

    def _draw_stations(self) -> None:
        for slot in self._slots:
            key = (slot.side, slot.role, slot.index)
            station = self._station_by_slot.get(key)
            color = L.STATION_COLORS[slot.role]
            label_prefix = L.ROLE_LABELS[slot.role]

            p0 = self._to_widget(slot.x - slot.w / 2, slot.y - slot.h / 2)
            p1 = self._to_widget(slot.x + slot.w / 2, slot.y + slot.h / 2)
            outline = TEXT if station is not None else MUTED
            dash = () if station is not None else (3, 3)
            rect_id = self.canvas.create_rectangle(
                *p0, *p1, fill=color if station is not None else '', outline=outline,
                width=2, dash=dash, tags=('station',),
            )

            display_id = station.station_id if station is not None else '?'
            top = f'{label_prefix} {display_id}'
            cp = self._to_widget(slot.x, slot.y)
            self.canvas.create_text(cp[0], cp[1] - 6, text=top, fill='#0b1220' if station else MUTED,
                                     font=('Segoe UI', 9, 'bold'))

            if station is not None:
                mats = list(station.material_ids)
                mat_text = ','.join(str(m) for m in mats[:6]) + ('…' if len(mats) > 6 else '')
                self.canvas.create_text(cp[0], cp[1] + 9, text=(mat_text or '-'),
                                         fill='#0b1220', font=('Consolas', 8))

            if station is not None and self._on_select is not None:
                name = str(station.name)
                mats = list(station.material_ids)
                self.canvas.tag_bind(
                    rect_id, '<Button-1>',
                    lambda _e, n=name, r=slot.role, m=mats: self._on_select(n, r, m),
                )

    def _draw_amr(self) -> None:
        if self._amr_xy is None:
            return
        x, y = self._to_widget(*self._amr_xy)
        color = AMR_COLOR if self._amr_live else MUTED
        r = 8
        self.canvas.create_oval(x - r, y - r, x + r, y + r, fill=color, outline=TEXT, width=2, tags='amr')
        self.canvas.create_text(x, y - 16, text='AMR', fill=color, font=('Segoe UI', 8, 'bold'), tags='amr')
