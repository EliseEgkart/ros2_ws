#!/usr/bin/env python3
"""
Fixed ENTRY TIER lifecycle mission publisher.

Arena layout station IDs (post arena-layout revision):
    Side A: start=0, storage_1=1, storage_2=2, hybrid_1=3, workbench_1=4
            (workbench_2=5 unused), customer_1=6, shared_storage_1=7
    Side B: start=14, storage_1=13, storage_2=12, hybrid_1=11, workbench_1=10
            (workbench_2=9 unused), customer_1=8, shared_storage_1=7 (shared)

Start-area positions (0 / 14) are robot spawn points, not physical
stations, so they are intentionally omitted from arena_layout.
"""

from typing import Dict, List

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from sml_messages.msg import Order, Station, Task


def make_order(order_type: int, name: str, product_id: int) -> Order:
    order = Order()
    order.order_type = order_type
    order.name = name
    order.product_id = product_id
    return order


def make_station(station_type: int, name: str, station_id: int, material_ids: List[int]) -> Station:
    station = Station()
    station.station_type = station_type
    station.name = name
    station.station_id = station_id
    station.material_ids = material_ids
    return station


def safe_material_ids(storage_materials: Dict[str, List[int]], key: str) -> List[int]:
    return list(storage_materials.get(key) or [])


def build_arena_layout(storage_materials: Dict[str, List[int]]) -> List[Station]:
    return [
        make_station(Station.ST_STORAGE, 'side_a_storage_1', 1, safe_material_ids(storage_materials, 'side_a_storage_1')),
        make_station(Station.ST_STORAGE, 'side_a_storage_2', 2, safe_material_ids(storage_materials, 'side_a_storage_2')),
        make_station(Station.ST_HYBRID, 'side_a_hybrid_1', 3, safe_material_ids(storage_materials, 'side_a_hybrid_1')),
        make_station(Station.ST_WORKBENCH, 'side_a_workbench_1', 4, []),
        make_station(Station.ST_CUSTOMER, 'side_a_customer_1', 6, safe_material_ids(storage_materials, 'side_a_customer_1')),

        make_station(Station.ST_STORAGE, 'shared_storage_1', 7, safe_material_ids(storage_materials, 'shared_storage_1')),

        make_station(Station.ST_CUSTOMER, 'side_b_customer_1', 8, safe_material_ids(storage_materials, 'side_b_customer_1')),
        make_station(Station.ST_WORKBENCH, 'side_b_workbench_1', 10, []),
        make_station(Station.ST_HYBRID, 'side_b_hybrid_1', 11, safe_material_ids(storage_materials, 'side_b_hybrid_1')),
        make_station(Station.ST_STORAGE, 'side_b_storage_2', 12, safe_material_ids(storage_materials, 'side_b_storage_2')),
        make_station(Station.ST_STORAGE, 'side_b_storage_1', 13, safe_material_ids(storage_materials, 'side_b_storage_1')),
    ]


def fill_task(orders: List[Order], storage_materials: Dict[str, List[int]]) -> Task:
    task = Task()
    task.order_list = orders
    task.arena_layout = build_arena_layout(storage_materials)
    return task


def build_side_only_task(task: Task, side: str) -> Task:
    side_prefix = f'{side}_'
    side_task = Task()
    side_task.order_list = list(task.order_list)
    side_task.arena_layout = [
        station for station in task.arena_layout
        if station.name.startswith(side_prefix) or station.name.startswith('shared_')
    ]
    return side_task


def build_lifecycle_entry_task() -> Task:
    orders = [
        make_order(Order.OT_PRODUCE, 'produce_battery', 34),
        make_order(Order.OT_PRODUCE, 'produce_estop', 81),
        make_order(Order.OT_RECYCLE, 'recycle_magnet', 13),
    ]
    return fill_task(
        orders,
        {
            'side_a_storage_1': [1, 8],
            'side_a_storage_2': [3, 4],
            'side_b_storage_1': [1, 8],
            'side_b_storage_2': [3, 4],
            'side_a_customer_1': [13],
            'side_b_customer_1': [13],
        },
    )


class LifecycleEntryServerNode(Node):
    def __init__(self) -> None:
        super().__init__('lifecycle_entry_server')

        self.declare_parameter('topic_name', '/eai/task')
        self.declare_parameter('side_a_topic_name', '/eai/task/side_a')
        self.declare_parameter('side_b_topic_name', '/eai/task/side_b')
        self.declare_parameter('publish_period_sec', 1.0)
        self.declare_parameter('publish_once', True)

        topic_name = self.get_parameter('topic_name').get_parameter_value().string_value
        side_a_topic_name = self.get_parameter('side_a_topic_name').get_parameter_value().string_value
        side_b_topic_name = self.get_parameter('side_b_topic_name').get_parameter_value().string_value
        period = self.get_parameter('publish_period_sec').get_parameter_value().double_value
        self._publish_once = self.get_parameter('publish_once').get_parameter_value().bool_value
        self._shutdown_requested = False
        self._task = build_lifecycle_entry_task()

        self._publisher = self.create_publisher(Task, topic_name, 10)
        self._side_a_publisher = self.create_publisher(Task, side_a_topic_name, 10)
        self._side_b_publisher = self.create_publisher(Task, side_b_topic_name, 10)
        self._timer = self.create_timer(period, self._publish_task)

        self.get_logger().info(
            f'Publishing fixed ENTRY TIER lifecycle task on {topic_name} every {period:.2f}s '
            f'(publish_once={self._publish_once})'
        )
        self.get_logger().info(f'Publishing side-specific task for side_a on {side_a_topic_name}')
        self.get_logger().info(f'Publishing side-specific task for side_b on {side_b_topic_name}')

    def _publish_task(self) -> None:
        side_a_task = build_side_only_task(self._task, 'side_a')
        side_b_task = build_side_only_task(self._task, 'side_b')

        self._publisher.publish(self._task)
        self._side_a_publisher.publish(side_a_task)
        self._side_b_publisher.publish(side_b_task)

        self.get_logger().info(
            f'Published task: orders={len(self._task.order_list)}, '
            f'stations={len(self._task.arena_layout)}'
        )
        self.get_logger().info(
            f'Published side_a task: orders={len(side_a_task.order_list)}, '
            f'stations={len(side_a_task.arena_layout)}'
        )
        self.get_logger().info(
            f'Published side_b task: orders={len(side_b_task.order_list)}, '
            f'stations={len(side_b_task.arena_layout)}'
        )

        if self._publish_once:
            self.get_logger().info('Published one task. Shutting down.')
            self._timer.cancel()
            self._shutdown_requested = True


def main(args=None) -> None:
    rclpy.init(args=args)
    node = LifecycleEntryServerNode()

    try:
        while rclpy.ok() and not node._shutdown_requested:
            rclpy.spin_once(node, timeout_sec=0.1)
    except ExternalShutdownException:
        pass
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
