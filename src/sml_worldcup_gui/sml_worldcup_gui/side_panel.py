"""
Side panel: order list, live interface-health indicators, and the currently
selected station's details. A local copy of the product-name table is kept
here (matching robocup_planner/product_catalog.py) rather than importing
that package, so the GUI keeps working even if robocup_planner fails to
build — the same decoupling precedent eai_task_server/order.py already uses
for its own product table.
"""

from __future__ import annotations

import tkinter as tk
from tkinter import ttk
from typing import List

from sml_messages.msg import Order, Task

from .ros_bridge import HealthStatus

PRODUCT_NAMES = {
    81: 'E-Stop', 34: 'Battery', 13: 'Magnet', 442: 'Carrot',
    241: 'Traffic Light', 462: 'Small Tree', 4482: 'Big Carrot', 711: 'Hammer',
    8518: 'Burger', 46262: 'Big Tree', 48132: 'Ice Cream',
}

TEXT = '#e5edf8'
MUTED = '#94a3b8'
GOOD = '#34d399'
BAD = '#f87171'
PANEL_BG = '#111827'


class SidePanel:
    def __init__(self, parent: tk.Widget):
        self.frame = tk.Frame(parent, bg=PANEL_BG, width=300)
        self.frame.pack(side=tk.RIGHT, fill=tk.Y)
        self.frame.pack_propagate(False)

        self._build_health_section()
        self._build_orders_section()
        self._build_station_section()

    # ------------------------------------------------------------------
    def _section_label(self, text: str) -> None:
        tk.Label(self.frame, text=text, bg=PANEL_BG, fg=MUTED,
                  font=('Segoe UI', 9, 'bold')).pack(anchor='w', padx=10, pady=(12, 2))

    def _build_health_section(self) -> None:
        self._section_label('CONNECTIONS')
        self._health_labels = {}
        grid = tk.Frame(self.frame, bg=PANEL_BG)
        grid.pack(fill=tk.X, padx=10)
        for key, title in (
            ('task', 'TASK'), ('odom', 'AMR ODOM'), ('nav', 'NAVIGATOR'),
            ('wb', 'WORKBENCH'), ('arm', 'ARM'), ('post', 'POST-PROCESS'),
        ):
            row = tk.Frame(grid, bg=PANEL_BG)
            row.pack(fill=tk.X, pady=1)
            dot = tk.Label(row, text='●', bg=PANEL_BG, fg=BAD, font=('Segoe UI', 10))
            dot.pack(side=tk.LEFT)
            tk.Label(row, text=title, bg=PANEL_BG, fg=TEXT, font=('Segoe UI', 9)).pack(side=tk.LEFT, padx=(6, 0))
            val = tk.Label(row, text='', bg=PANEL_BG, fg=MUTED, font=('Consolas', 9))
            val.pack(side=tk.RIGHT)
            self._health_labels[key] = (dot, val)

    def update_health(self, status: HealthStatus) -> None:
        mapping = {
            'task': (status.task_publishers > 0, str(status.task_publishers)),
            'odom': (status.odom_publishers > 0, str(status.odom_publishers)),
            'nav': (status.nav_action_up, 'up' if status.nav_action_up else 'down'),
            'wb': (status.wb_action_up, 'up' if status.wb_action_up else 'down'),
            'arm': (status.arm_service_up, 'up' if status.arm_service_up else 'down'),
            'post': (status.post_process_service_up, 'up' if status.post_process_service_up else 'down'),
        }
        for key, (ok, text) in mapping.items():
            dot, val = self._health_labels[key]
            dot.configure(fg=GOOD if ok else BAD)
            val.configure(text=text)

    # ------------------------------------------------------------------
    def _build_orders_section(self) -> None:
        self._section_label('ORDERS')
        columns = ('type', 'id', 'name')
        self._order_tree = ttk.Treeview(self.frame, columns=columns, show='headings', height=10)
        for col, width in (('type', 70), ('id', 55), ('name', 130)):
            self._order_tree.heading(col, text=col.upper())
            self._order_tree.column(col, width=width, anchor='w')
        self._order_tree.pack(fill=tk.X, padx=10, pady=(0, 6))

        self._order_summary = tk.Label(self.frame, text='no task received yet', bg=PANEL_BG, fg=MUTED,
                                        font=('Segoe UI', 8), justify='left', wraplength=280)
        self._order_summary.pack(anchor='w', padx=10)

    def update_task(self, task: Task) -> None:
        self._order_tree.delete(*self._order_tree.get_children())
        produce = 0
        recycle = 0
        for order in task.order_list:
            pid = int(order.product_id)
            name = PRODUCT_NAMES.get(pid, f'id={pid}')
            if int(order.order_type) == Order.OT_PRODUCE:
                kind = 'PRODUCE'
                produce += 1
            else:
                kind = 'RECYCLE'
                recycle += 1
            self._order_tree.insert('', tk.END, values=(kind, pid, name))

        self._order_summary.configure(
            text=f'{len(task.order_list)} orders  ({produce} produce / {recycle} recycle)  '
                 f'|  {len(task.arena_layout)} stations'
        )

    # ------------------------------------------------------------------
    def _build_station_section(self) -> None:
        self._section_label('SELECTED STATION')
        self._station_text = tk.Label(self.frame, text='(click a station)', bg=PANEL_BG, fg=TEXT,
                                       font=('Consolas', 9), justify='left', wraplength=280, anchor='nw')
        self._station_text.pack(fill=tk.X, padx=10, pady=(0, 10))

    def show_station(self, name: str, role: str, materials: List[int]) -> None:
        mat_str = ', '.join(str(m) for m in materials) if materials else '(empty)'
        self._station_text.configure(text=f'name: {name}\nrole: {role}\nmaterial_ids: [{mat_str}]')
