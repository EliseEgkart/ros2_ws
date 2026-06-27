import argparse
import copy
import math
import re
import shutil
import sys
from dataclasses import dataclass
from datetime import datetime
from pathlib import Path
from typing import Dict, Iterable, List, Optional, Tuple

import tkinter as tk
from tkinter import filedialog, messagebox, ttk

import yaml
from PIL import Image, ImageOps, ImageTk


STATION_WAYPOINT_RE = re.compile(r'^station_(\d+)_(sub_)?goal$')
DEFAULT_MAP_PATH = Path.home() / 'ros2_ws/src/amr/map/robocup_map.yaml'
DEFAULT_WAYPOINTS_PATH = (
    Path.home()
    / 'ros2_ws/src/robocup_navigator/params/robocup_waypoint.yaml'
)


@dataclass
class MapInfo:
    yaml_path: Path
    image_path: Path
    resolution: float
    origin: Tuple[float, float, float]
    image: Image.Image

    @property
    def width(self) -> int:
        return self.image.width

    @property
    def height(self) -> int:
        return self.image.height


def load_yaml(path: Path) -> dict:
    with path.open('r', encoding='utf-8') as stream:
        data = yaml.safe_load(stream) or {}
    if not isinstance(data, dict):
        raise ValueError(f'{path} must contain a YAML mapping.')
    return data


def load_map(map_yaml_path: Path) -> MapInfo:
    data = load_yaml(map_yaml_path)
    image_name = data.get('image')
    if not image_name:
        raise ValueError(f'{map_yaml_path} does not define image.')

    image_path = Path(image_name)
    if not image_path.is_absolute():
        image_path = map_yaml_path.parent / image_path

    resolution = float(data['resolution'])
    origin_data = data.get('origin', [0.0, 0.0, 0.0])
    if len(origin_data) < 3:
        origin_data = list(origin_data) + [0.0] * (3 - len(origin_data))
    origin = (
        float(origin_data[0]),
        float(origin_data[1]),
        float(origin_data[2]),
    )

    image = Image.open(image_path).convert('L')
    if int(data.get('negate', 0)):
        image = ImageOps.invert(image)

    return MapInfo(
        yaml_path=map_yaml_path,
        image_path=image_path,
        resolution=resolution,
        origin=origin,
        image=image,
    )


def quaternion_to_yaw(orientation: dict) -> float:
    x = float(orientation.get('x', 0.0))
    y = float(orientation.get('y', 0.0))
    z = float(orientation.get('z', 0.0))
    w = float(orientation.get('w', 1.0))
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    return math.atan2(siny_cosp, cosy_cosp)


def yaw_to_quaternion(yaw: float) -> dict:
    return {
        'x': 0.0,
        'y': 0.0,
        'z': round(math.sin(yaw / 2.0), 6),
        'w': round(math.cos(yaw / 2.0), 6),
    }


def normalize_degrees(value: float) -> float:
    while value > 180.0:
        value -= 360.0
    while value <= -180.0:
        value += 360.0
    return value


def waypoint_pose(x: float, y: float, yaw: float) -> dict:
    return {
        'position': {
            'x': round(x, 6),
            'y': round(y, 6),
            'z': 0.0,
        },
        'orientation': yaw_to_quaternion(yaw),
    }


def waypoint_xy_yaw(entry: dict) -> Tuple[float, float, float]:
    position = entry.get('position', {})
    orientation = entry.get('orientation', {})
    return (
        float(position.get('x', 0.0)),
        float(position.get('y', 0.0)),
        quaternion_to_yaw(orientation),
    )


def station_key_sort(value):
    try:
        return 0, int(value)
    except (TypeError, ValueError):
        return 1, str(value)


class WaypointEditor:
    def __init__(self, root: tk.Tk, map_info: MapInfo,
                 waypoints_path: Path):
        self.root = root
        self.map_info = map_info
        self.waypoints_path = waypoints_path
        self.data = load_yaml(waypoints_path)
        self.original_data = copy.deepcopy(self.data)

        self.data.setdefault('waypoints', {})
        self.data.setdefault('sequence', [])
        self.data.setdefault('stations', {})
        self.data.setdefault('frame_id', 'map')

        self.zoom = 2.0
        self.selected_name: Optional[str] = None
        self.drag_start_map: Optional[Tuple[float, float]] = None
        self.dirty = False
        self.photo = None
        self.map_image_item = None
        self.measure_start_map: Optional[Tuple[float, float]] = None
        self.measure_end_map: Optional[Tuple[float, float]] = None
        self.diff_start_map: Optional[Tuple[float, float]] = None
        self.diff_end_map: Optional[Tuple[float, float]] = None
        self.undo_stack = []
        self.redo_stack = []
        self.max_undo_states = 80

        self.name_var = tk.StringVar()
        self.station_id_var = tk.StringVar(value='1')
        self.kind_var = tk.StringVar(value='goal')
        self.x_var = tk.StringVar()
        self.y_var = tk.StringVar()
        self.yaw_var = tk.StringVar()
        self.frame_id_var = tk.StringVar(value=str(self.data['frame_id']))
        self.post_process_var = tk.BooleanVar(value=True)
        self.status_var = tk.StringVar()
        self.show_grid_var = tk.BooleanVar(value=True)
        self.snap_grid_var = tk.BooleanVar(value=False)
        self.grid_spacing_var = tk.StringVar(value='0.50')
        self.mode_var = tk.StringVar(value='edit')
        self.diff_toggle_var = tk.BooleanVar(value=False)
        self.diff_auto_next_var = tk.BooleanVar(value=True)
        self.robot_preview_var = tk.BooleanVar(value=False)
        self.robot_preview_mode_var = tk.StringVar(value='selected')
        self.robot_size_x_var = tk.StringVar(value='0.795')
        self.robot_size_y_var = tk.StringVar(value='0.600')
        self.robot_offset_x_var = tk.StringVar(value='0.00')
        self.robot_offset_y_var = tk.StringVar(value='0.00')

        self.root.title('Robocup Waypoint Editor')
        self._build_ui()
        self._bind_events()
        self._refresh_image()
        self._refresh_waypoint_list()
        self._draw_overlays()
        self._update_title()
        self._set_status('Loaded map and waypoint YAML.')

    @property
    def waypoints(self) -> Dict[str, dict]:
        return self.data.setdefault('waypoints', {})

    @property
    def stations(self) -> Dict:
        return self.data.setdefault('stations', {})

    @property
    def sequence(self) -> List[str]:
        return self.data.setdefault('sequence', [])

    def _build_ui(self):
        self.root.geometry('1180x760')

        main = ttk.Frame(self.root)
        main.pack(fill=tk.BOTH, expand=True)

        sidebar = ttk.Frame(main, padding=8)
        sidebar.pack(side=tk.LEFT, fill=tk.Y)

        toolbar = ttk.Frame(sidebar)
        toolbar.pack(fill=tk.X)
        ttk.Button(toolbar, text='Save', command=self.save).pack(
            side=tk.LEFT,
            fill=tk.X,
            expand=True,
            padx=(0, 4),
        )
        ttk.Button(toolbar, text='Save As', command=self.save_as).pack(
            side=tk.LEFT,
            fill=tk.X,
            expand=True,
        )
        ttk.Button(toolbar, text='Undo', command=self.undo).pack(
            side=tk.LEFT,
            fill=tk.X,
            expand=True,
            padx=(4, 0),
        )
        ttk.Button(toolbar, text='Redo', command=self.redo).pack(
            side=tk.LEFT,
            fill=tk.X,
            expand=True,
            padx=(4, 0),
        )

        ttk.Label(sidebar, text='Waypoints').pack(anchor=tk.W, pady=(12, 2))
        list_frame = ttk.Frame(sidebar)
        list_frame.pack(fill=tk.BOTH, expand=True)
        self.listbox = tk.Listbox(list_frame, width=34, height=18,
                                  exportselection=False)
        list_scroll = ttk.Scrollbar(list_frame, orient=tk.VERTICAL,
                                    command=self.listbox.yview)
        self.listbox.configure(yscrollcommand=list_scroll.set)
        self.listbox.pack(side=tk.LEFT, fill=tk.BOTH, expand=True)
        list_scroll.pack(side=tk.RIGHT, fill=tk.Y)

        form = ttk.LabelFrame(sidebar, text='Edit', padding=8)
        form.pack(fill=tk.X, pady=(8, 0))

        self._entry_row(form, 'Name', self.name_var, 0)
        self._entry_row(form, 'X', self.x_var, 1)
        self._entry_row(form, 'Y', self.y_var, 2)
        self._entry_row(form, 'Yaw deg', self.yaw_var, 3)
        self._entry_row(form, 'Frame', self.frame_id_var, 4)

        form.columnconfigure(1, weight=1)

        button_grid = ttk.Frame(form)
        button_grid.grid(row=5, column=0, columnspan=2, sticky=tk.EW,
                         pady=(6, 0))
        for index in range(2):
            button_grid.columnconfigure(index, weight=1)

        ttk.Button(
            button_grid,
            text='Apply Fields',
            command=self.apply_fields,
        ).grid(row=0, column=0, sticky=tk.EW, padx=(0, 3), pady=2)
        ttk.Button(
            button_grid,
            text='New Custom',
            command=self.new_custom_waypoint,
        ).grid(row=0, column=1, sticky=tk.EW, padx=(3, 0), pady=2)
        ttk.Button(
            button_grid,
            text='Delete',
            command=self.delete_selected,
        ).grid(row=1, column=0, sticky=tk.EW, padx=(0, 3), pady=2)
        ttk.Button(
            button_grid,
            text='Reload',
            command=self.reload,
        ).grid(row=1, column=1, sticky=tk.EW, padx=(3, 0), pady=2)

        tools = ttk.LabelFrame(sidebar, text='Tools', padding=8)
        tools.pack(fill=tk.X, pady=(8, 0))
        ttk.Label(
            tools,
            text='Mode',
        ).grid(row=0, column=0, sticky=tk.W, pady=2)
        self.mode_combo = ttk.Combobox(
            tools,
            textvariable=self.mode_var,
            values=('edit', 'measure', 'drag'),
            state='readonly',
            width=10,
        )
        self.mode_combo.grid(row=0, column=1, sticky=tk.EW, pady=2)
        ttk.Label(
            tools,
            text='Grid m',
        ).grid(row=1, column=0, sticky=tk.W, pady=2)
        self.grid_spacing_combo = ttk.Combobox(
            tools,
            textvariable=self.grid_spacing_var,
            values=('0.05', '0.10', '0.25', '0.50', '1.00'),
            width=10,
        )
        self.grid_spacing_combo.grid(row=1, column=1, sticky=tk.EW, pady=2)
        ttk.Checkbutton(
            tools,
            text='Show grid',
            variable=self.show_grid_var,
            command=self._draw_overlays,
        ).grid(row=2, column=0, columnspan=2, sticky=tk.W, pady=2)
        ttk.Checkbutton(
            tools,
            text='Snap to grid',
            variable=self.snap_grid_var,
        ).grid(row=3, column=0, columnspan=2, sticky=tk.W, pady=2)

        tool_buttons = ttk.Frame(tools)
        tool_buttons.grid(row=4, column=0, columnspan=2, sticky=tk.EW,
                          pady=(6, 0))
        tool_buttons.columnconfigure(0, weight=1)
        tool_buttons.columnconfigure(1, weight=1)
        ttk.Button(
            tool_buttons,
            text='Clear Measure',
            command=self.clear_measurement,
        ).grid(row=0, column=0, sticky=tk.EW, padx=(0, 3))
        ttk.Button(
            tool_buttons,
            text='Center',
            command=self.center_selected,
        ).grid(row=0, column=1, sticky=tk.EW, padx=(3, 0))
        tools.columnconfigure(1, weight=1)

        help_text = (
            'Left click: place selected waypoint\n'
            'Left drag: set heading\n'
            'Right click: select nearest waypoint\n'
            'Measure mode: drag distance ruler\n'
            'Drag mode: left drag pans map\n'
            'Wheel: zoom, middle drag: pan\n'
            'Alt+arrows: nudge, Alt+Q/E: rotate\n'
            'Ctrl+S: save, Del: delete'
        )
        ttk.Label(sidebar, text=help_text, justify=tk.LEFT).pack(
            anchor=tk.W,
            pady=(10, 0),
        )

        right_panel = ttk.Frame(main, padding=8)
        right_panel.pack(side=tk.RIGHT, fill=tk.Y)
        self._build_right_panel(right_panel)

        canvas_area = ttk.Frame(main)
        canvas_area.pack(side=tk.RIGHT, fill=tk.BOTH, expand=True)

        canvas_toolbar = ttk.Frame(canvas_area, padding=(0, 8, 8, 4))
        canvas_toolbar.pack(fill=tk.X)
        ttk.Button(canvas_toolbar, text='-', width=3,
                   command=lambda: self.set_zoom(self.zoom / 1.25)).pack(
            side=tk.LEFT,
            padx=(8, 2),
        )
        ttk.Button(canvas_toolbar, text='+', width=3,
                   command=lambda: self.set_zoom(self.zoom * 1.25)).pack(
            side=tk.LEFT,
            padx=2,
        )
        ttk.Button(canvas_toolbar, text='Fit',
                   command=self.fit_to_window).pack(side=tk.LEFT, padx=2)
        ttk.Label(
            canvas_toolbar,
            text=f'Map: {self.map_info.yaml_path}',
        ).pack(side=tk.LEFT, padx=(12, 0))

        canvas_frame = ttk.Frame(canvas_area)
        canvas_frame.pack(fill=tk.BOTH, expand=True)
        self.canvas = tk.Canvas(
            canvas_frame,
            bg='#202124',
            highlightthickness=0,
        )
        h_scroll = ttk.Scrollbar(canvas_frame, orient=tk.HORIZONTAL,
                                 command=self.canvas.xview)
        v_scroll = ttk.Scrollbar(canvas_frame, orient=tk.VERTICAL,
                                 command=self.canvas.yview)
        self.canvas.configure(
            xscrollcommand=h_scroll.set,
            yscrollcommand=v_scroll.set,
        )
        self.canvas.grid(row=0, column=0, sticky=tk.NSEW)
        v_scroll.grid(row=0, column=1, sticky=tk.NS)
        h_scroll.grid(row=1, column=0, sticky=tk.EW)
        canvas_frame.columnconfigure(0, weight=1)
        canvas_frame.rowconfigure(0, weight=1)

        status = ttk.Label(self.root, textvariable=self.status_var,
                           anchor=tk.W, relief=tk.SUNKEN)
        status.pack(side=tk.BOTTOM, fill=tk.X)

    def _build_right_panel(self, parent):
        parent.configure(width=180)
        competition_box = ttk.LabelFrame(parent, text='Competition Points',
                                         padding=8)
        competition_box.pack(fill=tk.X)

        station_row = ttk.Frame(competition_box)
        station_row.pack(fill=tk.X)
        ttk.Label(station_row, text='Station').pack(side=tk.LEFT)
        ttk.Spinbox(
            station_row,
            from_=1,
            to=20,
            width=6,
            textvariable=self.station_id_var,
        ).pack(side=tk.RIGHT)
        ttk.Checkbutton(
            competition_box,
            text='post_process',
            variable=self.post_process_var,
        ).pack(anchor=tk.W, pady=(6, 0))

        nav_buttons = ttk.Frame(competition_box)
        nav_buttons.pack(fill=tk.X, pady=(6, 0))
        nav_buttons.columnconfigure(0, weight=1)
        nav_buttons.columnconfigure(1, weight=1)
        ttk.Button(
            nav_buttons,
            text='Prev',
            command=lambda: self._step_station(-1),
        ).grid(row=0, column=0, sticky=tk.EW, padx=(0, 3))
        ttk.Button(
            nav_buttons,
            text='Next',
            command=lambda: self._step_station(1),
        ).grid(row=0, column=1, sticky=tk.EW, padx=(3, 0))

        select_buttons = ttk.Frame(competition_box)
        select_buttons.pack(fill=tk.X, pady=(6, 0))
        select_buttons.columnconfigure(0, weight=1)
        select_buttons.columnconfigure(1, weight=1)
        ttk.Button(
            select_buttons,
            text='Select Sub',
            command=self.select_station_sub_goal,
        ).grid(row=0, column=0, sticky=tk.EW, padx=(0, 3))
        ttk.Button(
            select_buttons,
            text='Select Goal',
            command=self.select_station_goal,
        ).grid(row=0, column=1, sticky=tk.EW, padx=(3, 0))

        ttk.Button(
            competition_box,
            text='Create 1-20 Pairs',
            command=self.create_competition_set,
        ).pack(fill=tk.X, pady=(8, 0))

        diff_box = ttk.LabelFrame(parent, text='Diff Pair', padding=8)
        diff_box.pack(fill=tk.X, pady=(8, 0))
        ttk.Checkbutton(
            diff_box,
            text='Diff pair toggle',
            variable=self.diff_toggle_var,
            command=self._on_diff_toggle_changed,
        ).pack(anchor=tk.W)
        ttk.Checkbutton(
            diff_box,
            text='Auto next station',
            variable=self.diff_auto_next_var,
        ).pack(anchor=tk.W, pady=(4, 0))
        ttk.Label(
            diff_box,
            text=(
                'Turn on this toggle,\n'
                'then drag:\n\n'
                'start: station_N_sub_goal\n'
                'end: station_N_goal\n'
                'yaw: start -> end'
            ),
            justify=tk.LEFT,
        ).pack(anchor=tk.W, pady=(8, 0))
        ttk.Button(
            diff_box,
            text='Clear Preview',
            command=self._clear_diff_preview,
        ).pack(fill=tk.X, pady=(8, 0))

        robot_box = ttk.LabelFrame(parent, text='Robot Preview', padding=8)
        robot_box.pack(fill=tk.X, pady=(8, 0))
        ttk.Checkbutton(
            robot_box,
            text='Show robot footprint',
            variable=self.robot_preview_var,
            command=self._draw_overlays,
        ).pack(anchor=tk.W)

        self._right_entry_row(robot_box, 'Size X m', self.robot_size_x_var)
        self._right_entry_row(robot_box, 'Size Y m', self.robot_size_y_var)
        self._right_entry_row(robot_box, 'Offset X m', self.robot_offset_x_var)
        self._right_entry_row(robot_box, 'Offset Y m', self.robot_offset_y_var)

        ttk.Label(robot_box, text='Targets').pack(anchor=tk.W, pady=(6, 2))
        self.robot_preview_mode_combo = ttk.Combobox(
            robot_box,
            textvariable=self.robot_preview_mode_var,
            values=('selected', 'point list', 'all'),
            state='readonly',
            width=14,
        )
        self.robot_preview_mode_combo.pack(fill=tk.X)
        self.robot_preview_mode_combo.bind(
            '<<ComboboxSelected>>',
            lambda _event: self._draw_overlays(),
        )

        list_frame = ttk.Frame(robot_box)
        list_frame.pack(fill=tk.BOTH, pady=(6, 0))
        self.robot_point_listbox = tk.Listbox(
            list_frame,
            height=8,
            selectmode=tk.EXTENDED,
            exportselection=False,
        )
        point_scroll = ttk.Scrollbar(
            list_frame,
            orient=tk.VERTICAL,
            command=self.robot_point_listbox.yview,
        )
        self.robot_point_listbox.configure(yscrollcommand=point_scroll.set)
        self.robot_point_listbox.pack(side=tk.LEFT, fill=tk.BOTH, expand=True)
        point_scroll.pack(side=tk.RIGHT, fill=tk.Y)
        self.robot_point_listbox.bind(
            '<<ListboxSelect>>',
            lambda _event: self._draw_overlays(),
        )

        robot_buttons = ttk.Frame(robot_box)
        robot_buttons.pack(fill=tk.X, pady=(6, 0))
        robot_buttons.columnconfigure(0, weight=1)
        robot_buttons.columnconfigure(1, weight=1)
        ttk.Button(
            robot_buttons,
            text='Select All',
            command=self._select_all_robot_points,
        ).grid(row=0, column=0, sticky=tk.EW, padx=(0, 3))
        ttk.Button(
            robot_buttons,
            text='Update',
            command=self._draw_overlays,
        ).grid(row=0, column=1, sticky=tk.EW, padx=(3, 0))

    def _right_entry_row(self, parent, label: str, variable: tk.StringVar):
        row = ttk.Frame(parent)
        row.pack(fill=tk.X, pady=2)
        ttk.Label(row, text=label).pack(side=tk.LEFT)
        ttk.Entry(row, textvariable=variable, width=8).pack(side=tk.RIGHT)

    def _entry_row(self, parent, label: str, variable: tk.StringVar, row: int):
        ttk.Label(
            parent,
            text=label,
        ).grid(row=row, column=0, sticky=tk.W, pady=2)
        ttk.Entry(
            parent,
            textvariable=variable,
        ).grid(row=row, column=1, sticky=tk.EW, pady=2)

    def _bind_events(self):
        self.listbox.bind('<<ListboxSelect>>', self._on_list_select)
        self.canvas.bind('<ButtonPress-1>', self._on_left_press)
        self.canvas.bind('<B1-Motion>', self._on_left_drag)
        self.canvas.bind('<ButtonRelease-1>', self._on_left_release)
        self.canvas.bind('<ButtonPress-3>', self._on_right_press)
        self.canvas.bind('<ButtonPress-2>', self._on_middle_press)
        self.canvas.bind('<B2-Motion>', self._on_middle_drag)
        self.canvas.bind('<MouseWheel>', self._on_mouse_wheel)
        self.canvas.bind('<Button-4>', self._on_mouse_wheel)
        self.canvas.bind('<Button-5>', self._on_mouse_wheel)
        self.canvas.bind('<Motion>', self._on_motion)
        self.mode_combo.bind('<<ComboboxSelected>>', self._on_mode_changed)
        self.grid_spacing_combo.bind(
            '<<ComboboxSelected>>',
            lambda _event: self._draw_overlays(),
        )
        self.root.bind('<Control-s>', lambda _event: self.save())
        self.root.bind('<Control-z>', lambda _event: self.undo())
        self.root.bind('<Control-Z>', lambda _event: self.redo())
        self.root.bind('<Control-y>', lambda _event: self.redo())
        self.root.bind('<Delete>', lambda _event: self.delete_selected())
        self.root.bind(
            '<Alt-Left>',
            lambda _event: self._nudge_selected(-1, 0),
        )
        self.root.bind(
            '<Alt-Right>',
            lambda _event: self._nudge_selected(1, 0),
        )
        self.root.bind('<Alt-Up>', lambda _event: self._nudge_selected(0, 1))
        self.root.bind(
            '<Alt-Down>',
            lambda _event: self._nudge_selected(0, -1),
        )
        self.root.bind('<Alt-q>', lambda _event: self._rotate_selected(5.0))
        self.root.bind('<Alt-e>', lambda _event: self._rotate_selected(-5.0))
        self.root.protocol('WM_DELETE_WINDOW', self._on_close)

    def _refresh_image(self):
        width = max(1, int(self.map_info.width * self.zoom))
        height = max(1, int(self.map_info.height * self.zoom))
        resampling = getattr(Image, 'Resampling', Image).NEAREST
        image = self.map_info.image.resize((width, height), resampling)
        self.photo = ImageTk.PhotoImage(image)
        if self.map_image_item is None:
            self.map_image_item = self.canvas.create_image(
                0,
                0,
                image=self.photo,
                anchor=tk.NW,
                tags=('map',),
            )
        else:
            self.canvas.itemconfigure(self.map_image_item, image=self.photo)
        self.canvas.configure(scrollregion=(0, 0, width, height))

    def _refresh_waypoint_list(self):
        selected = self.selected_name
        ordered_names = self._ordered_waypoint_names()
        self.listbox.delete(0, tk.END)
        for name in ordered_names:
            self.listbox.insert(tk.END, name)

        if selected in ordered_names:
            index = ordered_names.index(selected)
            self.listbox.selection_clear(0, tk.END)
            self.listbox.selection_set(index)
            self.listbox.see(index)
        self._refresh_robot_point_list(ordered_names)

    def _refresh_robot_point_list(self, ordered_names: List[str]):
        if not hasattr(self, 'robot_point_listbox'):
            return
        selected_names = {
            self.robot_point_listbox.get(index)
            for index in self.robot_point_listbox.curselection()
        }
        self.robot_point_listbox.delete(0, tk.END)
        for name in ordered_names:
            self.robot_point_listbox.insert(tk.END, name)
            if name in selected_names:
                self.robot_point_listbox.selection_set(tk.END)

    def _ordered_waypoint_names(self) -> List[str]:
        names = []
        for name in self.sequence:
            if name in self.waypoints and name not in names:
                names.append(name)

        for key in sorted(self.stations.keys(), key=station_key_sort):
            station = self.stations.get(key) or {}
            for name in station.get('sequence', []) or []:
                if name in self.waypoints and name not in names:
                    names.append(name)

        for name in self.waypoints:
            if name not in names:
                names.append(name)
        return names

    def _draw_overlays(self):
        self.canvas.delete('grid')
        self.canvas.delete('overlay')
        self.canvas.delete('measure')
        self.canvas.delete('diff_preview')
        self.canvas.delete('robot_preview')
        self._draw_grid()
        self._draw_sequence()
        self._draw_robot_previews()
        for name in self._ordered_waypoint_names():
            self._draw_waypoint(name)
        self._draw_measurement()
        self._draw_diff_preview()

    def _draw_grid(self):
        if not self.show_grid_var.get():
            return

        spacing = self._grid_spacing()
        if spacing <= 0.0:
            return

        min_x, min_y, max_x, max_y = self._map_bounds()
        start_x = math.floor(min_x / spacing) * spacing
        start_y = math.floor(min_y / spacing) * spacing
        major_every = max(1, int(round(1.0 / spacing)))

        index = 0
        x = start_x
        while x <= max_x + spacing:
            major = index % major_every == 0
            color = '#3f5f6f' if major else '#2d3f48'
            width = 1 if major else 1
            x1, y1 = self.map_to_canvas(x, min_y)
            x2, y2 = self.map_to_canvas(x, max_y)
            self.canvas.create_line(
                x1,
                y1,
                x2,
                y2,
                fill=color,
                width=width,
                tags=('grid',),
            )
            if major:
                lx, ly = self.map_to_canvas(x, min_y)
                self.canvas.create_text(
                    lx + 2,
                    ly - 2,
                    text=f'{x:.1f}',
                    anchor=tk.SW,
                    fill='#24424f',
                    font=('TkDefaultFont', 8),
                    tags=('grid',),
                )
            x += spacing
            index += 1

        index = 0
        y = start_y
        while y <= max_y + spacing:
            major = index % major_every == 0
            color = '#3f5f6f' if major else '#2d3f48'
            x1, y1 = self.map_to_canvas(min_x, y)
            x2, y2 = self.map_to_canvas(max_x, y)
            self.canvas.create_line(
                x1,
                y1,
                x2,
                y2,
                fill=color,
                tags=('grid',),
            )
            if major:
                lx, ly = self.map_to_canvas(min_x, y)
                self.canvas.create_text(
                    lx + 2,
                    ly - 2,
                    text=f'{y:.1f}',
                    anchor=tk.SW,
                    fill='#24424f',
                    font=('TkDefaultFont', 8),
                    tags=('grid',),
                )
            y += spacing
            index += 1

    def _map_bounds(self) -> Tuple[float, float, float, float]:
        corners = (
            self.pixel_to_map(0.0, 0.0),
            self.pixel_to_map(float(self.map_info.width), 0.0),
            self.pixel_to_map(float(self.map_info.width),
                              float(self.map_info.height)),
            self.pixel_to_map(0.0, float(self.map_info.height)),
        )
        xs = [point[0] for point in corners]
        ys = [point[1] for point in corners]
        return min(xs), min(ys), max(xs), max(ys)

    def _grid_spacing(self) -> float:
        try:
            spacing = float(self.grid_spacing_var.get())
        except ValueError:
            spacing = 0.5
        return max(0.01, spacing)

    def _draw_sequence(self):
        points = []
        for name in self.sequence:
            entry = self.waypoints.get(name)
            if not entry:
                continue
            x, y, _yaw = waypoint_xy_yaw(entry)
            points.append(self.map_to_canvas(x, y))

        for start, end in zip(points, points[1:]):
            self.canvas.create_line(
                start[0],
                start[1],
                end[0],
                end[1],
                fill='#2f80ed',
                width=2,
                dash=(6, 4),
                tags=('overlay',),
            )

    def _draw_waypoint(self, name: str):
        entry = self.waypoints[name]
        x, y, yaw = waypoint_xy_yaw(entry)
        cx, cy = self.map_to_canvas(x, y)
        length_m = max(0.35, 20.0 / self.zoom * self.map_info.resolution)
        ex, ey = self.map_to_canvas(
            x + math.cos(yaw) * length_m,
            y + math.sin(yaw) * length_m,
        )
        selected = name == self.selected_name
        fill = '#f2994a' if selected else '#27ae60'
        outline = '#ffffff' if selected else '#0b3d24'
        radius = 6 if selected else 5

        self.canvas.create_line(
            cx,
            cy,
            ex,
            ey,
            fill=fill,
            width=3 if selected else 2,
            arrow=tk.LAST,
            arrowshape=(12, 14, 5),
            tags=('overlay',),
        )
        self.canvas.create_oval(
            cx - radius,
            cy - radius,
            cx + radius,
            cy + radius,
            fill=fill,
            outline=outline,
            width=2,
            tags=('overlay',),
        )
        self.canvas.create_text(
            cx + 8,
            cy - 8,
            text=name,
            anchor=tk.SW,
            fill='#111111',
            font=('TkDefaultFont', 9, 'bold' if selected else 'normal'),
            tags=('overlay',),
        )

    def _draw_robot_previews(self):
        if not self.robot_preview_var.get():
            return

        try:
            size_x = max(0.01, float(self.robot_size_x_var.get()))
            size_y = max(0.01, float(self.robot_size_y_var.get()))
            offset_x = float(self.robot_offset_x_var.get())
            offset_y = float(self.robot_offset_y_var.get())
        except ValueError:
            self._set_status(
                'Robot preview ignored: size/offset must be numeric.'
            )
            return

        for name in self._robot_preview_names():
            entry = self.waypoints.get(name)
            if not entry:
                continue
            x, y, yaw = waypoint_xy_yaw(entry)
            self._draw_robot_footprint(
                name,
                x,
                y,
                yaw,
                size_x,
                size_y,
                offset_x,
                offset_y,
            )

    def _robot_preview_names(self) -> List[str]:
        mode = self.robot_preview_mode_var.get()
        if mode == 'all':
            return self._ordered_waypoint_names()
        if mode == 'point list' and hasattr(self, 'robot_point_listbox'):
            return [
                self.robot_point_listbox.get(index)
                for index in self.robot_point_listbox.curselection()
            ]
        if self.selected_name:
            return [self.selected_name]
        return []

    def _draw_robot_footprint(self, name: str, x: float, y: float, yaw: float,
                              size_x: float, size_y: float, offset_x: float,
                              offset_y: float):
        center_x, center_y = self._local_to_map(x, y, yaw, offset_x, offset_y)
        local_corners = (
            (size_x / 2.0, size_y / 2.0),
            (size_x / 2.0, -size_y / 2.0),
            (-size_x / 2.0, -size_y / 2.0),
            (-size_x / 2.0, size_y / 2.0),
        )
        points = []
        for local_x, local_y in local_corners:
            corner = self._local_to_map(
                center_x,
                center_y,
                yaw,
                local_x,
                local_y,
            )
            points.extend(self.map_to_canvas(corner[0], corner[1]))

        self.canvas.create_polygon(
            *points,
            fill='',
            outline='#8e44ad',
            width=2,
            tags=('robot_preview',),
        )
        cx, cy = self.map_to_canvas(center_x, center_y)
        fx, fy = self._local_to_map(center_x, center_y, yaw, size_x / 2.0, 0.0)
        fcx, fcy = self.map_to_canvas(fx, fy)
        wx, wy = self.map_to_canvas(x, y)
        self.canvas.create_line(
            cx,
            cy,
            fcx,
            fcy,
            fill='#8e44ad',
            width=2,
            arrow=tk.LAST,
            tags=('robot_preview',),
        )
        self.canvas.create_line(
            wx,
            wy,
            cx,
            cy,
            fill='#8e44ad',
            width=1,
            dash=(3, 3),
            tags=('robot_preview',),
        )
        self.canvas.create_text(
            cx + 6,
            cy + 6,
            text=f'robot:{name}',
            anchor=tk.NW,
            fill='#8e44ad',
            font=('TkDefaultFont', 8, 'bold'),
            tags=('robot_preview',),
        )

    def _local_to_map(self, base_x: float, base_y: float, yaw: float,
                      local_x: float, local_y: float) -> Tuple[float, float]:
        cos_yaw = math.cos(yaw)
        sin_yaw = math.sin(yaw)
        return (
            base_x + cos_yaw * local_x - sin_yaw * local_y,
            base_y + sin_yaw * local_x + cos_yaw * local_y,
        )

    def _draw_measurement(self):
        if not self.measure_start_map or not self.measure_end_map:
            return

        start_x, start_y = self.measure_start_map
        end_x, end_y = self.measure_end_map
        x1, y1 = self.map_to_canvas(start_x, start_y)
        x2, y2 = self.map_to_canvas(end_x, end_y)
        dx = end_x - start_x
        dy = end_y - start_y
        distance = math.hypot(dx, dy)
        angle = normalize_degrees(math.degrees(math.atan2(dy, dx)))
        label = (
            f'{distance:.3f} m  dx={dx:.3f}  dy={dy:.3f}  yaw={angle:.1f} deg'
        )

        self.canvas.create_line(
            x1,
            y1,
            x2,
            y2,
            fill='#d00000',
            width=3,
            tags=('measure',),
        )
        for cx, cy in ((x1, y1), (x2, y2)):
            self.canvas.create_oval(
                cx - 5,
                cy - 5,
                cx + 5,
                cy + 5,
                fill='#ffffff',
                outline='#d00000',
                width=2,
                tags=('measure',),
            )

        mid_x = (x1 + x2) / 2.0
        mid_y = (y1 + y2) / 2.0
        text_id = self.canvas.create_text(
            mid_x + 10,
            mid_y - 10,
            text=label,
            anchor=tk.SW,
            fill='#d00000',
            font=('TkDefaultFont', 10, 'bold'),
            tags=('measure',),
        )
        bbox = self.canvas.bbox(text_id)
        if bbox:
            background = self.canvas.create_rectangle(
                bbox[0] - 3,
                bbox[1] - 2,
                bbox[2] + 3,
                bbox[3] + 2,
                fill='#fff7d6',
                outline='#d00000',
                tags=('measure',),
            )
            self.canvas.tag_lower(background, text_id)

    def _draw_diff_preview(self):
        if not self.diff_start_map or not self.diff_end_map:
            return

        start_x, start_y = self.diff_start_map
        end_x, end_y = self.diff_end_map
        x1, y1 = self.map_to_canvas(start_x, start_y)
        x2, y2 = self.map_to_canvas(end_x, end_y)
        dx = end_x - start_x
        dy = end_y - start_y
        distance = math.hypot(dx, dy)
        yaw = normalize_degrees(math.degrees(math.atan2(dy, dx)))
        label = f'diff pair  dist={distance:.3f} m  yaw={yaw:.1f} deg'

        self.canvas.create_line(
            x1,
            y1,
            x2,
            y2,
            fill='#b00020',
            width=3,
            arrow=tk.LAST,
            arrowshape=(14, 16, 6),
            tags=('diff_preview',),
        )
        self._draw_diff_marker(x1, y1, 'SUB')
        self._draw_diff_marker(x2, y2, 'GOAL')

        text_id = self.canvas.create_text(
            (x1 + x2) / 2.0 + 10,
            (y1 + y2) / 2.0 - 10,
            text=label,
            anchor=tk.SW,
            fill='#b00020',
            font=('TkDefaultFont', 10, 'bold'),
            tags=('diff_preview',),
        )
        bbox = self.canvas.bbox(text_id)
        if bbox:
            background = self.canvas.create_rectangle(
                bbox[0] - 3,
                bbox[1] - 2,
                bbox[2] + 3,
                bbox[3] + 2,
                fill='#ffe8e8',
                outline='#b00020',
                tags=('diff_preview',),
            )
            self.canvas.tag_lower(background, text_id)

    def _draw_diff_marker(self, x: float, y: float, label: str):
        self.canvas.create_oval(
            x - 6,
            y - 6,
            x + 6,
            y + 6,
            fill='#ffffff',
            outline='#b00020',
            width=2,
            tags=('diff_preview',),
        )
        self.canvas.create_text(
            x + 8,
            y + 8,
            text=label,
            anchor=tk.NW,
            fill='#b00020',
            font=('TkDefaultFont', 9, 'bold'),
            tags=('diff_preview',),
        )

    def map_to_canvas(self, x: float, y: float) -> Tuple[float, float]:
        pixel_x, pixel_y = self.map_to_pixel(x, y)
        return pixel_x * self.zoom, pixel_y * self.zoom

    def canvas_to_map(self, canvas_x: float,
                      canvas_y: float) -> Tuple[float, float]:
        return self.pixel_to_map(canvas_x / self.zoom, canvas_y / self.zoom)

    def map_to_pixel(self, x: float, y: float) -> Tuple[float, float]:
        ox, oy, yaw = self.map_info.origin
        dx = x - ox
        dy = y - oy
        cos_yaw = math.cos(yaw)
        sin_yaw = math.sin(yaw)
        local_x = cos_yaw * dx + sin_yaw * dy
        local_y = -sin_yaw * dx + cos_yaw * dy
        return (
            local_x / self.map_info.resolution,
            self.map_info.height - local_y / self.map_info.resolution,
        )

    def pixel_to_map(self, pixel_x: float,
                     pixel_y: float) -> Tuple[float, float]:
        local_x = pixel_x * self.map_info.resolution
        local_y = (self.map_info.height - pixel_y) * self.map_info.resolution
        ox, oy, yaw = self.map_info.origin
        cos_yaw = math.cos(yaw)
        sin_yaw = math.sin(yaw)
        return (
            ox + cos_yaw * local_x - sin_yaw * local_y,
            oy + sin_yaw * local_x + cos_yaw * local_y,
        )

    def _on_list_select(self, _event=None):
        selection = self.listbox.curselection()
        if not selection:
            return
        name = self.listbox.get(selection[0])
        self.select_waypoint(name)

    def select_waypoint(self, name: str):
        if name not in self.waypoints:
            return
        self.selected_name = name
        self.name_var.set(name)
        x, y, yaw = waypoint_xy_yaw(self.waypoints[name])
        self.x_var.set(f'{x:.6f}')
        self.y_var.set(f'{y:.6f}')
        self.yaw_var.set(f'{normalize_degrees(math.degrees(yaw)):.3f}')
        self._populate_station_fields(name)
        self._refresh_waypoint_list()
        self._draw_overlays()
        self._set_status(f'Selected {name}.')

    def _populate_station_fields(self, name: str):
        match = STATION_WAYPOINT_RE.match(name)
        if not match:
            self.kind_var.set('custom')
            return
        self.station_id_var.set(match.group(1))
        self.kind_var.set('sub_goal' if match.group(2) else 'goal')
        station = self.stations.get(int(match.group(1)))
        if station is None:
            station = self.stations.get(match.group(1), {})
        self.post_process_var.set(bool(station.get('post_process', True)))

    def new_station_waypoint(self):
        try:
            station_id = int(self.station_id_var.get())
        except ValueError:
            messagebox.showerror('Invalid station', 'Station must be an int.')
            return

        kind = self.kind_var.get()
        if kind == 'custom':
            kind = 'goal'
        suffix = 'sub_goal' if kind == 'sub_goal' else 'goal'
        name = f'station_{station_id}_{suffix}'
        if name in self.waypoints:
            self.select_waypoint(name)
            return
        self._push_undo()
        self._create_or_select_waypoint(name)
        self._ensure_station_membership(name, update_post_process=True)
        self.mark_dirty()
        self.select_waypoint(name)

    def select_station_sub_goal(self):
        self.kind_var.set('sub_goal')
        self.new_station_waypoint()

    def select_station_goal(self):
        self.kind_var.set('goal')
        self.new_station_waypoint()

    def _step_station(self, delta: int):
        station_id = self._current_station_id()
        if station_id is None:
            return
        station_id = min(20, max(1, station_id + delta))
        self.station_id_var.set(str(station_id))
        self._set_status(f'Station set to {station_id}.')

    def _current_station_id(self) -> Optional[int]:
        try:
            return int(self.station_id_var.get())
        except ValueError:
            messagebox.showerror('Invalid station', 'Station must be an int.')
            return None

    def create_competition_set(self):
        self._push_undo()
        center_x, center_y = self.pixel_to_map(
            self.map_info.width / 2.0,
            self.map_info.height / 2.0,
        )
        created = 0
        ordered_names = []
        for station_id in range(1, 21):
            for suffix in ('sub_goal', 'goal'):
                name = f'station_{station_id}_{suffix}'
                ordered_names.append(name)
                if name not in self.waypoints:
                    self.waypoints[name] = waypoint_pose(
                        center_x,
                        center_y,
                        0.0,
                    )
                    created += 1
                self._ensure_station_membership(
                    name,
                    update_post_process=False,
                )

        extras = [
            name for name in self.sequence
            if name in self.waypoints and name not in ordered_names
        ]
        self.data['sequence'] = ordered_names + extras

        self.station_id_var.set('1')
        self.kind_var.set('sub_goal')
        self.selected_name = None
        self.mark_dirty()
        self.select_waypoint('station_1_sub_goal')
        self._set_status(
            f'Competition set ready: station_1..20 pairs, '
            f'{created} new waypoints.'
        )

    def new_custom_waypoint(self):
        self._push_undo()
        base = self.name_var.get().strip() or 'waypoint'
        base = re.sub(r'[^A-Za-z0-9_]+', '_', base).strip('_') or 'waypoint'
        name = base
        index = 1
        while name in self.waypoints:
            name = f'{base}_{index}'
            index += 1
        self._create_or_select_waypoint(name)
        self.sequence.append(name)
        self.mark_dirty()
        self.select_waypoint(name)

    def _create_or_select_waypoint(self, name: str):
        if name in self.waypoints:
            self.selected_name = name
            return

        center_x, center_y = self.pixel_to_map(
            self.map_info.width / 2.0,
            self.map_info.height / 2.0,
        )
        self.waypoints[name] = waypoint_pose(center_x, center_y, 0.0)
        if name not in self.sequence:
            self.sequence.append(name)
        self.selected_name = name

    def apply_fields(self):
        if not self.selected_name:
            messagebox.showinfo('No waypoint', 'Select or create a waypoint.')
            return False
        old_name = self.selected_name
        new_name = self.name_var.get().strip()
        if not new_name:
            messagebox.showerror('Invalid name', 'Waypoint name is required.')
            return False
        try:
            x = float(self.x_var.get())
            y = float(self.y_var.get())
            yaw = math.radians(float(self.yaw_var.get()))
        except ValueError:
            messagebox.showerror('Invalid pose', 'X, Y, and yaw must be nums.')
            return False
        if new_name != old_name and new_name in self.waypoints:
            messagebox.showerror(
                'Duplicate name',
                f'{new_name} already exists.',
            )
            return False

        self._push_undo()
        if new_name != old_name:
            self._rename_waypoint(old_name, new_name)
        self.waypoints[new_name] = waypoint_pose(x, y, yaw)
        self.data['frame_id'] = self.frame_id_var.get().strip() or 'map'
        self._ensure_station_membership(new_name, update_post_process=True)
        self.selected_name = new_name
        self.mark_dirty()
        self._refresh_waypoint_list()
        self._draw_overlays()
        self._set_status(f'Applied {new_name}.')
        return True

    def _rename_waypoint(self, old_name: str, new_name: str):
        self.waypoints[new_name] = self.waypoints.pop(old_name)
        self._replace_references(self.sequence, old_name, new_name)
        for station in self.stations.values():
            if isinstance(station, dict):
                self._replace_references(
                    station.setdefault('sequence', []),
                    old_name,
                    new_name,
                )
        self.selected_name = new_name

    def _replace_references(self, items: List[str], old_name: str,
                            new_name: str):
        for index, item in enumerate(list(items)):
            if item == old_name:
                items[index] = new_name

    def _ensure_station_membership(self, waypoint_name: str,
                                   update_post_process: bool = False):
        match = STATION_WAYPOINT_RE.match(waypoint_name)
        if not match:
            return

        station_id = int(match.group(1))
        station = self.stations.get(station_id)
        if station is None:
            station = self.stations.get(str(station_id))
        if station is None:
            station = {
                'name': f'station_{station_id}',
                'sequence': [],
                'post_process': (
                    bool(self.post_process_var.get())
                    if update_post_process
                    else station_id != 0
                ),
            }
            self.stations[station_id] = station

        station['name'] = station.get('name') or f'station_{station_id}'
        if update_post_process:
            station['post_process'] = bool(self.post_process_var.get())
        else:
            station['post_process'] = bool(
                station.get('post_process', station_id != 0)
            )
        sequence = station.setdefault('sequence', [])
        if waypoint_name not in sequence:
            if match.group(2):
                goal_name = f'station_{station_id}_goal'
                if goal_name in sequence:
                    sequence.insert(sequence.index(goal_name), waypoint_name)
                else:
                    sequence.append(waypoint_name)
            else:
                sequence.append(waypoint_name)

        ordered = []
        sub_name = f'station_{station_id}_sub_goal'
        goal_name = f'station_{station_id}_goal'
        for candidate in (sub_name, goal_name):
            if candidate in sequence:
                ordered.append(candidate)
        for candidate in sequence:
            if candidate not in ordered:
                ordered.append(candidate)
        station['sequence'] = ordered

        for name in station['sequence']:
            if name in self.waypoints and name not in self.sequence:
                self.sequence.append(name)

    def delete_selected(self):
        name = self.selected_name
        if not name:
            return
        if not messagebox.askyesno('Delete waypoint', f'Delete {name}?'):
            return
        self._push_undo()
        self.waypoints.pop(name, None)
        self.data['sequence'] = [
            item for item in self.sequence if item != name
        ]
        for station in self.stations.values():
            if isinstance(station, dict):
                station['sequence'] = [
                    item for item in station.get('sequence', [])
                    if item != name
                ]
        self.selected_name = None
        self.name_var.set('')
        self.x_var.set('')
        self.y_var.set('')
        self.yaw_var.set('')
        self.mark_dirty()
        self._refresh_waypoint_list()
        self._draw_overlays()
        self._set_status(f'Deleted {name}.')

    def _on_left_press(self, event):
        mode = self._active_mode()
        if mode == 'drag':
            self.canvas.scan_mark(event.x, event.y)
            return
        if mode == 'diff':
            self.diff_start_map = self._event_to_map(event, snap=True)
            self.diff_end_map = self.diff_start_map
            self._draw_overlays()
            self._set_status(
                'Diff mode: drag from sub_goal to goal, then release.'
            )
            return
        if mode == 'measure':
            self.measure_start_map = self._event_to_map(event)
            self.measure_end_map = self.measure_start_map
            self._draw_overlays()
            return

        name = self._ensure_selected_for_canvas()
        if not name:
            return
        x, y = self._event_to_map(event, snap=True)
        self._push_undo()
        _old_x, _old_y, yaw = waypoint_xy_yaw(self.waypoints[name])
        self.waypoints[name] = waypoint_pose(x, y, yaw)
        self.drag_start_map = (x, y)
        self._sync_fields_from_pose(name)
        self.mark_dirty()
        self._draw_overlays()

    def _on_left_drag(self, event):
        mode = self._active_mode()
        if mode == 'drag':
            self.canvas.scan_dragto(event.x, event.y, gain=1)
            return
        if mode == 'diff':
            if not self.diff_start_map:
                return
            self.diff_end_map = self._event_to_map(event, snap=True)
            self._draw_overlays()
            self._set_diff_status()
            return
        if mode == 'measure':
            if not self.measure_start_map:
                return
            self.measure_end_map = self._event_to_map(event)
            self._draw_overlays()
            self._set_measure_status()
            return

        name = self.selected_name
        if not name or not self.drag_start_map:
            return
        end_x, end_y = self._event_to_map(event)
        start_x, start_y = self.drag_start_map
        yaw = math.atan2(end_y - start_y, end_x - start_x)
        if math.hypot(end_x - start_x, end_y - start_y) < 0.03:
            return
        self.waypoints[name] = waypoint_pose(start_x, start_y, yaw)
        self._sync_fields_from_pose(name)
        self.mark_dirty()
        self._draw_overlays()

    def _on_left_release(self, event):
        mode = self._active_mode()
        if mode == 'drag':
            return
        if mode == 'diff':
            if self.diff_start_map:
                self.diff_end_map = self._event_to_map(event, snap=True)
                self._commit_diff_pair()
            return
        if mode == 'measure':
            if self.measure_start_map:
                self.measure_end_map = self._event_to_map(event)
                self._draw_overlays()
                self._set_measure_status()
            return
        self.drag_start_map = None

    def _on_right_press(self, event):
        x, y = self._event_to_map(event)
        nearest = self._nearest_waypoint(x, y)
        if nearest:
            self.select_waypoint(nearest)

    def _on_motion(self, event):
        x, y = self._event_to_map(event)
        if self.snap_grid_var.get():
            sx, sy = self._snap_to_grid(x, y)
            position = (
                f'map x={x:.3f}, y={y:.3f} | '
                f'snap x={sx:.3f}, y={sy:.3f}'
            )
        else:
            position = f'map x={x:.3f}, y={y:.3f}'
        mode = self._active_mode()
        self._set_status(f'{position} | mode={mode} | zoom={self.zoom:.2f}x')

    def _on_middle_press(self, event):
        self.canvas.scan_mark(event.x, event.y)

    def _on_middle_drag(self, event):
        self.canvas.scan_dragto(event.x, event.y, gain=1)

    def _on_mouse_wheel(self, event):
        if getattr(event, 'num', None) == 5 or getattr(event, 'delta', 0) < 0:
            factor = 1.0 / 1.2
        else:
            factor = 1.2
        self._zoom_at(event.x, event.y, self.zoom * factor)

    def _on_mode_changed(self, _event=None):
        mode = self.mode_var.get()
        if mode == 'measure':
            self._set_status('Measure mode: left drag between two map points.')
        elif mode == 'drag':
            self._set_status('Drag mode: left drag pans the map.')
        else:
            self._set_status('Edit mode: left click/drag edits waypoints.')

    def _ensure_selected_for_canvas(self) -> Optional[str]:
        if self.selected_name in self.waypoints:
            return self.selected_name

        if self.kind_var.get() == 'custom':
            self.new_custom_waypoint()
        else:
            self.new_station_waypoint()
        return self.selected_name

    def _event_to_map(self, event, snap: bool = False) -> Tuple[float, float]:
        canvas_x = self.canvas.canvasx(event.x)
        canvas_y = self.canvas.canvasy(event.y)
        x, y = self.canvas_to_map(canvas_x, canvas_y)
        if snap and self.snap_grid_var.get():
            return self._snap_to_grid(x, y)
        return x, y

    def _snap_to_grid(self, x: float, y: float) -> Tuple[float, float]:
        spacing = self._grid_spacing()
        return (
            round(x / spacing) * spacing,
            round(y / spacing) * spacing,
        )

    def clear_measurement(self):
        self.measure_start_map = None
        self.measure_end_map = None
        self._draw_overlays()
        self._set_status('Measurement cleared.')

    def _active_mode(self) -> str:
        if self.diff_toggle_var.get():
            return 'diff'
        return self.mode_var.get()

    def _on_diff_toggle_changed(self):
        if self.diff_toggle_var.get():
            self._set_status('Diff pair toggle on: left drag creates pair.')
        else:
            self._clear_diff_preview()
            self._set_status(
                f'Diff pair toggle off. Active mode={self.mode_var.get()}.'
            )

    def _clear_diff_preview(self):
        self.diff_start_map = None
        self.diff_end_map = None
        self._draw_overlays()

    def _select_all_robot_points(self):
        if not hasattr(self, 'robot_point_listbox'):
            return
        self.robot_preview_mode_var.set('point list')
        self.robot_point_listbox.selection_set(0, tk.END)
        self.robot_preview_var.set(True)
        self._draw_overlays()

    def _set_diff_status(self):
        if not self.diff_start_map or not self.diff_end_map:
            return
        start_x, start_y = self.diff_start_map
        end_x, end_y = self.diff_end_map
        dx = end_x - start_x
        dy = end_y - start_y
        distance = math.hypot(dx, dy)
        yaw = normalize_degrees(math.degrees(math.atan2(dy, dx)))
        self._set_status(
            f'diff pair preview | dist={distance:.3f} m, yaw={yaw:.1f} deg'
        )

    def _commit_diff_pair(self):
        if not self.diff_start_map or not self.diff_end_map:
            return

        try:
            station_id = int(self.station_id_var.get())
        except ValueError:
            messagebox.showerror('Invalid station', 'Station must be an int.')
            self.diff_start_map = None
            self.diff_end_map = None
            self._draw_overlays()
            return

        start_x, start_y = self.diff_start_map
        end_x, end_y = self.diff_end_map
        distance = math.hypot(end_x - start_x, end_y - start_y)
        if distance < 0.03:
            self._set_status('Diff pair ignored: drag distance is too short.')
            self.diff_start_map = None
            self.diff_end_map = None
            self._draw_overlays()
            return

        self._push_undo()
        yaw = math.atan2(end_y - start_y, end_x - start_x)
        sub_name = f'station_{station_id}_sub_goal'
        goal_name = f'station_{station_id}_goal'
        self.waypoints[sub_name] = waypoint_pose(start_x, start_y, yaw)
        self.waypoints[goal_name] = waypoint_pose(end_x, end_y, yaw)

        self._ensure_station_membership(sub_name, update_post_process=True)
        self._ensure_station_membership(goal_name, update_post_process=True)
        for name in (sub_name, goal_name):
            if name not in self.sequence:
                self.sequence.append(name)

        self.diff_start_map = None
        self.diff_end_map = None
        self.selected_name = goal_name
        self._sync_fields_from_pose(goal_name)
        self._populate_station_fields(goal_name)
        self.mark_dirty()
        self._refresh_waypoint_list()
        self._draw_overlays()
        self._set_status(
            f'Created {sub_name} -> {goal_name} '
            f'with yaw={normalize_degrees(math.degrees(yaw)):.1f} deg.'
        )
        if self.diff_auto_next_var.get():
            self._step_station(1)
            self._set_status(
                f'Created {sub_name} -> {goal_name}; '
                f'advanced to station {self.station_id_var.get()}.'
            )

    def _set_measure_status(self):
        if not self.measure_start_map or not self.measure_end_map:
            return
        start_x, start_y = self.measure_start_map
        end_x, end_y = self.measure_end_map
        dx = end_x - start_x
        dy = end_y - start_y
        distance = math.hypot(dx, dy)
        angle = normalize_degrees(math.degrees(math.atan2(dy, dx)))
        self._set_status(
            f'measure {distance:.3f} m | dx={dx:.3f}, dy={dy:.3f}, '
            f'yaw={angle:.1f} deg'
        )

    def center_selected(self):
        if not self.selected_name or self.selected_name not in self.waypoints:
            self._set_status('No selected waypoint to center.')
            return
        x, y, _yaw = waypoint_xy_yaw(self.waypoints[self.selected_name])
        canvas_x, canvas_y = self.map_to_canvas(x, y)
        self._center_canvas_at(canvas_x, canvas_y)
        self._set_status(f'Centered {self.selected_name}.')

    def _center_canvas_at(self, canvas_x: float, canvas_y: float):
        scrollregion = self.canvas.cget('scrollregion')
        if not scrollregion:
            return
        _left, _top, right, bottom = [
            float(value) for value in scrollregion.split()
        ]
        width = max(1.0, right)
        height = max(1.0, bottom)
        view_width = max(1, self.canvas.winfo_width())
        view_height = max(1, self.canvas.winfo_height())
        self.canvas.xview_moveto(
            self._clamp((canvas_x - view_width / 2.0) / width, 0.0, 1.0)
        )
        self.canvas.yview_moveto(
            self._clamp((canvas_y - view_height / 2.0) / height, 0.0, 1.0)
        )

    def _zoom_at(self, event_x: int, event_y: int, zoom: float):
        map_x, map_y = self.canvas_to_map(
            self.canvas.canvasx(event_x),
            self.canvas.canvasy(event_y),
        )
        self.set_zoom(zoom)
        canvas_x, canvas_y = self.map_to_canvas(map_x, map_y)
        self.canvas.xview_moveto(
            self._clamp(
                (
                    (canvas_x - event_x)
                    / max(1.0, self.map_info.width * self.zoom)
                ),
                0.0,
                1.0,
            )
        )
        self.canvas.yview_moveto(
            self._clamp(
                (canvas_y - event_y) / max(1.0,
                                           self.map_info.height * self.zoom),
                0.0,
                1.0,
            )
        )

    def _nudge_selected(self, x_dir: int, y_dir: int):
        if not self.selected_name or self.selected_name not in self.waypoints:
            return
        step = self._grid_spacing() if self.snap_grid_var.get() else 0.05
        x, y, yaw = waypoint_xy_yaw(self.waypoints[self.selected_name])
        self._push_undo()
        self.waypoints[self.selected_name] = waypoint_pose(
            x + x_dir * step,
            y + y_dir * step,
            yaw,
        )
        self._sync_fields_from_pose(self.selected_name)
        self.mark_dirty()
        self._draw_overlays()
        self._set_status(
            f'Nudged {self.selected_name} by {step:.3f} m.'
        )

    def _rotate_selected(self, delta_deg: float):
        if not self.selected_name or self.selected_name not in self.waypoints:
            return
        x, y, yaw = waypoint_xy_yaw(self.waypoints[self.selected_name])
        yaw += math.radians(delta_deg)
        self._push_undo()
        self.waypoints[self.selected_name] = waypoint_pose(x, y, yaw)
        self._sync_fields_from_pose(self.selected_name)
        self.mark_dirty()
        self._draw_overlays()
        self._set_status(
            f'Rotated {self.selected_name} by {delta_deg:.1f} deg.'
        )

    def _clamp(self, value: float, minimum: float, maximum: float) -> float:
        return max(minimum, min(maximum, value))

    def _nearest_waypoint(self, x: float, y: float) -> Optional[str]:
        best_name = None
        best_distance = float('inf')
        for name, entry in self.waypoints.items():
            wx, wy, _yaw = waypoint_xy_yaw(entry)
            distance = math.hypot(wx - x, wy - y)
            if distance < best_distance:
                best_name = name
                best_distance = distance
        threshold = 14.0 / self.zoom * self.map_info.resolution
        return best_name if best_distance <= threshold else None

    def _sync_fields_from_pose(self, name: str):
        x, y, yaw = waypoint_xy_yaw(self.waypoints[name])
        self.name_var.set(name)
        self.x_var.set(f'{x:.6f}')
        self.y_var.set(f'{y:.6f}')
        self.yaw_var.set(f'{normalize_degrees(math.degrees(yaw)):.3f}')

    def set_zoom(self, zoom: float):
        self.zoom = min(20.0, max(0.2, zoom))
        self._refresh_image()
        self._draw_overlays()

    def fit_to_window(self):
        self.root.update_idletasks()
        canvas_width = max(1, self.canvas.winfo_width())
        canvas_height = max(1, self.canvas.winfo_height())
        self.set_zoom(min(
            canvas_width / self.map_info.width,
            canvas_height / self.map_info.height,
        ))

    def mark_dirty(self):
        self.dirty = True
        self._update_title()

    def _update_title(self):
        dirty_mark = '*' if self.dirty else ''
        self.root.title(
            f'Robocup Waypoint Editor{dirty_mark} - {self.waypoints_path}'
        )

    def save_as(self):
        chosen = filedialog.asksaveasfilename(
            initialdir=str(self.waypoints_path.parent),
            initialfile=self.waypoints_path.name,
            defaultextension='.yaml',
            filetypes=(('YAML files', '*.yaml'), ('All files', '*.*')),
        )
        if not chosen:
            return
        self.waypoints_path = Path(chosen)
        self.save()

    def save(self):
        if self.selected_name:
            try:
                if not self.apply_fields():
                    return
            except tk.TclError:
                return

        output = self._normalized_output()
        if self.waypoints_path.exists():
            stamp = datetime.now().strftime('%Y%m%d-%H%M%S')
            backup_path = self.waypoints_path.with_suffix(
                self.waypoints_path.suffix + f'.bak-{stamp}'
            )
            shutil.copy2(self.waypoints_path, backup_path)

        with self.waypoints_path.open('w', encoding='utf-8') as stream:
            yaml.safe_dump(
                output,
                stream,
                allow_unicode=True,
                default_flow_style=False,
                sort_keys=False,
            )

        self.data = output
        self.original_data = copy.deepcopy(output)
        self.dirty = False
        self.redo_stack.clear()
        self._refresh_waypoint_list()
        self._draw_overlays()
        self._update_title()
        self._set_status(f'Saved {self.waypoints_path}.')

    def _normalized_output(self) -> dict:
        for name in list(self.waypoints.keys()):
            self._ensure_station_membership(name)

        ordered_waypoints = {}
        for name in self._ordered_waypoint_names():
            if name in self.waypoints:
                ordered_waypoints[name] = self.waypoints[name]

        sequence = self._unique_existing(self.sequence)
        stations = {}
        for key in sorted(self.stations.keys(), key=station_key_sort):
            station = self.stations[key]
            if not isinstance(station, dict):
                continue
            station_copy = dict(station)
            station_copy['sequence'] = self._unique_existing(
                station.get('sequence', [])
            )
            stations[key] = station_copy

        output = {}
        output['waypoints'] = ordered_waypoints
        output['sequence'] = sequence
        output['frame_id'] = self.frame_id_var.get().strip() or 'map'
        output['stations'] = stations
        return output

    def _unique_existing(self, names: Iterable[str]) -> List[str]:
        result = []
        for name in names:
            if name in self.waypoints and name not in result:
                result.append(name)
        return result

    def reload(self):
        if self.dirty and not messagebox.askyesno(
                'Reload',
                'Discard unsaved waypoint changes and reload from disk?'):
            return
        self.data = load_yaml(self.waypoints_path)
        self.data.setdefault('waypoints', {})
        self.data.setdefault('sequence', [])
        self.data.setdefault('stations', {})
        self.data.setdefault('frame_id', 'map')
        self.original_data = copy.deepcopy(self.data)
        self.frame_id_var.set(str(self.data['frame_id']))
        self.selected_name = None
        self.dirty = False
        self.undo_stack.clear()
        self.redo_stack.clear()
        self._refresh_waypoint_list()
        self._draw_overlays()
        self._update_title()
        self._set_status('Reloaded waypoint YAML.')

    def _set_status(self, text: str):
        self.status_var.set(text)

    def _snapshot_state(self) -> dict:
        return {
            'data': copy.deepcopy(self.data),
            'selected_name': self.selected_name,
            'frame_id': self.frame_id_var.get(),
            'station_id': self.station_id_var.get(),
            'kind': self.kind_var.get(),
            'post_process': self.post_process_var.get(),
        }

    def _restore_state(self, state: dict):
        self.data = copy.deepcopy(state['data'])
        self.data.setdefault('waypoints', {})
        self.data.setdefault('sequence', [])
        self.data.setdefault('stations', {})
        self.data.setdefault('frame_id', 'map')
        self.selected_name = state.get('selected_name')
        self.frame_id_var.set(
            state.get('frame_id', str(self.data['frame_id']))
        )
        self.station_id_var.set(state.get('station_id', '1'))
        self.kind_var.set(state.get('kind', 'goal'))
        self.post_process_var.set(bool(state.get('post_process', True)))
        self.diff_start_map = None
        self.diff_end_map = None
        self.measure_start_map = None
        self.measure_end_map = None
        self._refresh_waypoint_list()
        if self.selected_name in self.waypoints:
            self._sync_fields_from_pose(self.selected_name)
            self.name_var.set(self.selected_name)
            self._populate_station_fields(self.selected_name)
        else:
            self.selected_name = None
            self.name_var.set('')
            self.x_var.set('')
            self.y_var.set('')
            self.yaw_var.set('')
        self._draw_overlays()

    def _push_undo(self):
        self.undo_stack.append(self._snapshot_state())
        if len(self.undo_stack) > self.max_undo_states:
            self.undo_stack.pop(0)
        self.redo_stack.clear()

    def undo(self):
        if not self.undo_stack:
            self._set_status('Nothing to undo.')
            return
        self.redo_stack.append(self._snapshot_state())
        state = self.undo_stack.pop()
        self._restore_state(state)
        self.dirty = True
        self._update_title()
        self._set_status('Undo.')

    def redo(self):
        if not self.redo_stack:
            self._set_status('Nothing to redo.')
            return
        self.undo_stack.append(self._snapshot_state())
        state = self.redo_stack.pop()
        self._restore_state(state)
        self.dirty = True
        self._update_title()
        self._set_status('Redo.')

    def _on_close(self):
        if self.dirty and not messagebox.askyesno(
                'Unsaved changes',
                'Close without saving waypoint changes?'):
            return
        self.root.destroy()


def parse_args(argv: Optional[List[str]] = None):
    parser = argparse.ArgumentParser(
        description='GUI editor for ROS2 map waypoints.',
    )
    parser.add_argument(
        '--map',
        default=str(DEFAULT_MAP_PATH),
        help='Path to ROS map YAML.',
    )
    parser.add_argument(
        '--waypoints',
        default=str(DEFAULT_WAYPOINTS_PATH),
        help='Path to navigator waypoint YAML.',
    )
    return parser.parse_args(argv)


def main(argv: Optional[List[str]] = None):
    args = parse_args(argv)
    try:
        map_info = load_map(Path(args.map).expanduser().resolve())
        waypoints_path = Path(args.waypoints).expanduser().resolve()
        root = tk.Tk()
        WaypointEditor(root, map_info, waypoints_path)
        root.mainloop()
    except Exception as exc:
        print(f'waypoint_editor: {exc}', file=sys.stderr)
        return 1
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
