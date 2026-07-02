"""
tk.Canvas renderer for the arena.

Station positions and the AMR marker are both derived from real
robocup_waypoint.yaml meters through the SAME waypoints.FitTransform
instance (rebuilt whenever a new Task arrives, since the set of stations —
and therefore the bounding box being fit — can change). This is what
guarantees they line up: there is exactly one coordinate transform in play,
not two independently-calibrated ones.

Stations whose station_id has no entry in the waypoint file yet (it's a
work in progress — see waypoints.py) are listed in a small tray at the
bottom instead of being silently dropped or placed at a guessed position.
"""

from __future__ import annotations

import tkinter as tk
from typing import Callable, Dict, List, Optional, Tuple

from sml_messages.msg import Task

from . import layout_schema as L
from . import waypoints as W
from .station_roles import group_by_role, parse_role

BACKGROUND = '#0b1220'
TEXT = '#e5edf8'
MUTED = '#94a3b8'
AMR_COLOR = '#38bdf8'
TRAY_OUTLINE = '#475569'


class ArenaCanvas:
    def __init__(self, parent: tk.Widget, real_positions: Dict[int, Tuple[float, float]],
                 on_select: Optional[Callable[[str, str, List[int]], None]] = None):
        self.canvas = tk.Canvas(parent, bg=BACKGROUND, highlightthickness=0)
        self.canvas.pack(fill=tk.BOTH, expand=True)
        self._on_select = on_select
        self._real_positions = real_positions

        self.side_filter = 'all'  # 'a' | 'b' | 'all'
        self._resolved: List[dict] = []    # {name, role, side, station_id, materials, canvas_xy}
        self._unresolved: List[dict] = []  # same shape, canvas_xy omitted
        self._transform: Optional[W.FitTransform] = None
        self._side_bounds: Dict[str, Tuple[float, float, float, float]] = {}
        self._amr_real_xy: Optional[Tuple[float, float]] = None
        self._amr_live = False

        self.canvas.bind('<Configure>', lambda _e: self._redraw())

    # ------------------------------------------------------------------
    # Public API
    # ------------------------------------------------------------------

    def set_side_filter(self, side: str) -> None:
        self.side_filter = side
        self._redraw()

    def set_task(self, task: Task) -> None:
        resolved: List[dict] = []
        unresolved: List[dict] = []
        real_points: List[Tuple[float, float]] = []

        for st in task.arena_layout:
            info = parse_role(st)
            xy = W.resolve_station_xy(st.station_id, info.side, self._real_positions)
            entry = {
                'name': str(st.name), 'role': info.role, 'side': info.side,
                'station_id': int(st.station_id), 'materials': list(st.material_ids),
                'real_xy': xy,
            }
            if xy is not None:
                resolved.append(entry)
                real_points.append(xy)
            else:
                unresolved.append(entry)

        self._transform = W.fit_transform(real_points, L.ARENA_X0, L.ARENA_Y0, L.ARENA_X1, L.ARENA_Y1)
        if self._transform is not None:
            for entry in resolved:
                entry['canvas_xy'] = self._transform.apply(*entry['real_xy'])
        else:
            for entry in resolved:
                entry['canvas_xy'] = None

        self._resolved = resolved
        self._unresolved = unresolved
        self._side_bounds = self._compute_side_bounds()
        self._redraw()

    def set_amr_real_position(self, real_xy: Optional[Tuple[float, float]], live: bool) -> None:
        self._amr_real_xy = real_xy
        self._amr_live = live
        self._redraw_amr_only()

    # ------------------------------------------------------------------
    # Derived geometry
    # ------------------------------------------------------------------

    def _compute_side_bounds(self) -> Dict[str, Tuple[float, float, float, float]]:
        bounds: Dict[str, Tuple[float, float, float, float]] = {}
        for side in ('side_a', 'side_b'):
            pts = [e['canvas_xy'] for e in self._resolved if e['side'] == side and e['canvas_xy']]
            if len(pts) < 1:
                continue
            xs = [p[0] for p in pts]
            ys = [p[1] for p in pts]
            pad = L.STATION_BOX_W
            bounds[side] = (min(xs) - pad, min(ys) - pad, max(xs) + pad, max(ys) + pad)
        return bounds

    def _view_bounds(self) -> Tuple[float, float, float, float]:
        key = {'a': 'side_a', 'b': 'side_b'}.get(self.side_filter)
        if key and key in self._side_bounds:
            return self._side_bounds[key]
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
        self._draw_side_tints()
        self._draw_stations()
        self._draw_unresolved_tray()
        self._draw_amr()

    def _redraw_amr_only(self) -> None:
        self.canvas.delete('amr')
        self._draw_amr()

    def _draw_side_tints(self) -> None:
        for side, color, label in (
            ('side_a', L.ZONE_TINTS['side_a'], 'Side A'),
            ('side_b', L.ZONE_TINTS['side_b'], 'Side B'),
        ):
            if side not in self._side_bounds:
                continue
            x0, y0, x1, y1 = self._side_bounds[side]
            p0 = self._to_widget(x0, y0)
            p1 = self._to_widget(x1, y1)
            self.canvas.create_rectangle(*p0, *p1, fill=color, outline='', tags='zone')
            self.canvas.create_text(p0[0] + 8, p0[1] + 10, text=label, fill=TEXT,
                                     font=('Segoe UI', 11, 'bold'), anchor='w')

    def _draw_stations(self) -> None:
        for entry in self._resolved:
            if entry['canvas_xy'] is None:
                continue
            self._draw_one_station(entry, entry['canvas_xy'])

    def _draw_one_station(self, entry: dict, canvas_xy: Tuple[float, float]) -> None:
        color = L.STATION_COLORS.get(entry['role'], '#64748b')
        prefix = L.ROLE_LABELS.get(entry['role'], '?')
        x, y = canvas_xy
        p0 = self._to_widget(x - L.STATION_BOX_W / 2, y - L.STATION_BOX_H / 2)
        p1 = self._to_widget(x + L.STATION_BOX_W / 2, y + L.STATION_BOX_H / 2)
        rect_id = self.canvas.create_rectangle(*p0, *p1, fill=color, outline=TEXT, width=2, tags='station')

        cp = self._to_widget(x, y)
        self.canvas.create_text(cp[0], cp[1] - 6, text=f"{prefix} {entry['station_id']}",
                                 fill='#0b1220', font=('Segoe UI', 9, 'bold'))
        mats = entry['materials']
        mat_text = ','.join(str(m) for m in mats[:6]) + ('…' if len(mats) > 6 else '')
        self.canvas.create_text(cp[0], cp[1] + 9, text=(mat_text or '-'),
                                 fill='#0b1220', font=('Consolas', 8))

        if self._on_select is not None:
            name, role, mats2 = entry['name'], entry['role'], mats
            self.canvas.tag_bind(rect_id, '<Button-1>',
                                  lambda _e, n=name, r=role, m=mats2: self._on_select(n, r, m))

    def _draw_unresolved_tray(self) -> None:
        if not self._unresolved:
            return
        x = 20.0
        for entry in self._unresolved:
            p = self._to_widget(x, L.TRAY_Y)
            self.canvas.create_rectangle(p[0], p[1] - 10, p[0] + 100, p[1] + 10,
                                          outline=TRAY_OUTLINE, dash=(3, 3))
            self.canvas.create_text(p[0] + 50, p[1], text=f"{entry['name']} (no waypoint)",
                                     fill=MUTED, font=('Segoe UI', 7))
            x += 110.0
        note_p = self._to_widget(20.0, L.TRAY_Y - 20)
        self.canvas.create_text(note_p[0], note_p[1], anchor='w', fill=MUTED,
                                 font=('Segoe UI', 8, 'italic'),
                                 text='Stations below have no robocup_waypoint.yaml entry yet:')

    def _draw_amr(self) -> None:
        if self._amr_real_xy is None or self._transform is None:
            return
        canvas_xy = self._transform.apply(*self._amr_real_xy)
        x, y = self._to_widget(*canvas_xy)
        color = AMR_COLOR if self._amr_live else MUTED
        r = 8
        self.canvas.create_oval(x - r, y - r, x + r, y + r, fill=color, outline=TEXT, width=2, tags='amr')
        self.canvas.create_text(x, y - 16, text='AMR', fill=color, font=('Segoe UI', 8, 'bold'), tags='amr')
