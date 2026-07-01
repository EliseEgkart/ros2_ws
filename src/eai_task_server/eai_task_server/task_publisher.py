#!/usr/bin/env python3

from typing import Callable, Dict, Tuple

import rclpy
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException
from sml_messages.msg import Order, Station, Task


TaskBuilder = Callable[[], Task]


def safe_material_ids(storage_materials: Dict[str, list[int]], key: str) -> list[int]:
    value = storage_materials.get(key)
    if not value:
        return []
    return value


def build_mirrored_arena_layout(storage_materials: Dict[str, list[int]]) -> list[Station]:
    return [
        make_station(Station.ST_STORAGE, 'side_a_storage_1', 1, safe_material_ids(storage_materials, 'side_a_storage_1')),
        make_station(Station.ST_STORAGE, 'side_a_storage_2', 2, safe_material_ids(storage_materials, 'side_a_storage_2')),
        make_station(Station.ST_STORAGE, 'side_a_storage_3', 3, safe_material_ids(storage_materials, 'side_a_storage_3')),
        make_station(Station.ST_WORKBENCH, 'side_a_workbench_1', 4, []),
        make_station(Station.ST_WORKBENCH, 'side_a_workbench_2', 5, []),
        make_station(Station.ST_HYBRID, 'side_a_hybrid_1', 6, safe_material_ids(storage_materials, 'side_a_hybrid_1')),
        make_station(Station.ST_CUSTOMER, 'side_a_customer_1', 7, safe_material_ids(storage_materials, 'side_a_customer_1')),

        make_station(Station.ST_STORAGE, 'side_b_storage_1', 8, safe_material_ids(storage_materials, 'side_b_storage_1')),
        make_station(Station.ST_STORAGE, 'side_b_storage_2', 9, safe_material_ids(storage_materials, 'side_b_storage_2')),
        make_station(Station.ST_STORAGE, 'side_b_storage_3', 10, safe_material_ids(storage_materials, 'side_b_storage_3')),
        make_station(Station.ST_WORKBENCH, 'side_b_workbench_1', 11, []),
        make_station(Station.ST_WORKBENCH, 'side_b_workbench_2', 12, []),
        make_station(Station.ST_HYBRID, 'side_b_hybrid_1', 13, safe_material_ids(storage_materials, 'side_b_hybrid_1')),
        make_station(Station.ST_CUSTOMER, 'side_b_customer_1', 14, safe_material_ids(storage_materials, 'side_b_customer_1')),
    ]


def fill_task(orders: list[Order], storage_materials: Dict[str, list[int]]) -> Task:
    task = Task()
    task.order_list = orders
    task.arena_layout = build_mirrored_arena_layout(storage_materials)
    return task


def build_side_only_task(task: Task, side: str) -> Task:
    side_prefix = f'{side}_'

    side_task = Task()
    side_task.order_list = list(task.order_list)
    side_task.arena_layout = [
        station for station in task.arena_layout if station.name.startswith(side_prefix)
    ]
    return side_task


def make_order(order_type: int, name: str, product_id: int) -> Order:
    order = Order()
    order.order_type = order_type
    order.name = name
    order.product_id = product_id
    return order


def make_station(station_type: int, name: str, station_id: int, material_ids: list[int]) -> Station:
    station = Station()
    station.station_type = station_type
    station.name = name
    station.station_id = station_id
    station.material_ids = material_ids
    return station

### ENTRY TIER

def build_production_entry_task() -> Task:
    orders = [
        make_order(Order.OT_PRODUCE, 'produce_magnet', 13),
    ]
    return fill_task(
        orders,
        {
            'side_a_storage_1': [1],
            'side_a_storage_2': [3],
            'side_b_storage_1': [1],
            'side_b_storage_2': [3],
        },
    )

def build_recycling_entry_task() -> Task:
    orders = [
        make_order(Order.OT_RECYCLE, 'recycle_battery', 34),
    ]
    return fill_task(
        orders,
        {
            'side_a_storage_1': [4],
            'side_a_storage_2': [3],
            'side_b_storage_1': [4],
            'side_b_storage_2': [3],
            'side_a_customer_1': [34],
            'side_b_customer_1': [34],
        },
    )

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

### BEGINNER TIER

def build_production_beginner_task() -> Task:
    orders = [
        make_order(Order.OT_PRODUCE, 'produce_estop', 81),
        make_order(Order.OT_PRODUCE, 'produce_carrot', 442),
    ]
    return fill_task(
        orders,
        {
            'side_a_storage_1': [2, 1],
            'side_a_storage_2': [8],
            'side_a_storage_3': [40],
            'side_b_storage_1': [2, 1],
            'side_b_storage_2': [8],
            'side_b_storage_3': [40],
        },
    )

def build_recycling_beginner_task() -> Task:
    orders = [
        make_order(Order.OT_RECYCLE, 'recycle_magnet', 13),
        make_order(Order.OT_RECYCLE, 'recycle_traffic_light', 241),
    ]
    return fill_task(
        orders,
        {
            'side_a_storage_1': [1, 3],
            'side_a_storage_3': [4],
            'side_a_hybrid_1': [2],
            'side_b_storage_1': [1, 3],
            'side_b_storage_3': [4],
            'side_b_hybrid_1': [2],
        },
    )

def build_lifecycle_beginner_task() -> Task:
    orders = [
        make_order(Order.OT_PRODUCE, 'produce_magnet', 13),
        make_order(Order.OT_PRODUCE, 'produce_small_tree', 462),
        make_order(Order.OT_PRODUCE, 'produce_hammer', 711),
        make_order(Order.OT_RECYCLE, 'recycle_carrot', 442),
        make_order(Order.OT_RECYCLE, 'recycle_battery', 34),
    ]
    return fill_task(
        orders,
        {
            'side_a_storage_1': [10, 2],
            'side_a_storage_2': [4, 30],
            'side_a_storage_3': [6],
            'side_a_hybrid_1': [7],
            'side_a_customer_1': [34, 442],
            
            'side_b_storage_1': [10, 2],
            'side_b_storage_2': [4, 30],
            'side_b_storage_3': [6],
            'side_b_hybrid_1': [7],
            'side_b_customer_1': [34, 442],
        },
    )


### ADVANCED TIER

def build_production_advanced_task() -> Task:
    orders = [
        make_order(Order.OT_PRODUCE, 'pa_order_1', 3),
        make_order(Order.OT_PRODUCE, 'pa_order_2', 4),
        make_order(Order.OT_PRODUCE, 'pa_order_3', 5),
    ]
    return fill_task(
        orders,
        {
            'side_a_storage_1': [1, 2, 3],
            'side_a_storage_2': [4, 5],
            'side_b_storage_1': [2, 3, 4],
            'side_b_storage_2': [5, 6],
        },
    )

def build_recycling_advanced_task() -> Task:
    orders = [
        make_order(Order.OT_RECYCLE, 'ra_order_1', 3),
        make_order(Order.OT_RECYCLE, 'ra_order_2', 4),
        make_order(Order.OT_RECYCLE, 'ra_order_3', 5),
    ]
    return fill_task(
        orders,
        {
            'side_a_storage_1': [2, 3, 4],
            'side_a_storage_2': [5, 6],
            'side_b_storage_1': [1, 3, 5],
            'side_b_storage_2': [2, 4, 6],
        },
    )

def build_lifecycle_advanced_task() -> Task:
    # Exercises both recycle paths in one task:
    #  - recycle_magnet(13): also being produced this run, and there is no
    #    initial customer stock for it — the robot must assemble it, deliver
    #    it, then pick it back up for disassembly (deferred recycle path).
    #  - recycle_estop(81): pre-stocked on the customer counter (not being
    #    produced this run) — recycled immediately in Phase 1, as before.
    orders = [
        make_order(Order.OT_PRODUCE, 'la_produce_magnet', 13),
        make_order(Order.OT_PRODUCE, 'la_produce_battery', 34),
        make_order(Order.OT_RECYCLE, 'la_recycle_magnet_deferred', 13),
        make_order(Order.OT_RECYCLE, 'la_recycle_estop_immediate', 81),
    ]
    return fill_task(
        orders,
        {
            'side_a_storage_1': [1, 3],
            'side_a_storage_2': [3, 4],
            'side_a_customer_1': [81],
            'side_b_storage_1': [1, 3],
            'side_b_storage_2': [3, 4],
            'side_b_customer_1': [81],
        },
    )


TASK_BUILDERS: Dict[Tuple[str, str], TaskBuilder] = {
    ('production', 'entry'): build_production_entry_task,
    ('production', 'beginner'): build_production_beginner_task,
    ('production', 'advanced'): build_production_advanced_task,
    ('recycling', 'entry'): build_recycling_entry_task,
    ('recycling', 'beginner'): build_recycling_beginner_task,
    ('recycling', 'advanced'): build_recycling_advanced_task,
    ('lifecycle', 'entry'): build_lifecycle_entry_task,
    ('lifecycle', 'beginner'): build_lifecycle_beginner_task,
    ('lifecycle', 'advanced'): build_lifecycle_advanced_task,
}


class TaskPublisherNode(Node):
    def __init__(self) -> None:
        super().__init__('task_publisher')

        self.declare_parameter('scenario', 'production')
        self.declare_parameter('stage', 'beginner')
        self.declare_parameter('topic_name', '/eai/task')
        self.declare_parameter('side_a_topic_name', '/eai/task/side_a')
        self.declare_parameter('side_b_topic_name', '/eai/task/side_b')
        self.declare_parameter('publish_period_sec', 1.0)
        self.declare_parameter('publish_once', False)

        scenario = self.get_parameter('scenario').get_parameter_value().string_value.strip().lower()
        stage = self.get_parameter('stage').get_parameter_value().string_value.strip().lower()
        topic_name = self.get_parameter('topic_name').get_parameter_value().string_value
        side_a_topic_name = self.get_parameter('side_a_topic_name').get_parameter_value().string_value
        side_b_topic_name = self.get_parameter('side_b_topic_name').get_parameter_value().string_value
        period = self.get_parameter('publish_period_sec').get_parameter_value().double_value
        publish_once = self.get_parameter('publish_once').get_parameter_value().bool_value

        key = (scenario, stage)
        if key not in TASK_BUILDERS:
            valid = ', '.join([f'{k[0]}/{k[1]}' for k in TASK_BUILDERS.keys()])
            raise ValueError(f'Unsupported scenario/stage: {scenario}/{stage}. Valid options: {valid}')

        self._build_task = TASK_BUILDERS[key]
        self._publish_once = publish_once
        self._shutdown_requested = False
        self._publisher = self.create_publisher(Task, topic_name, 10)
        self._side_a_publisher = self.create_publisher(Task, side_a_topic_name, 10)
        self._side_b_publisher = self.create_publisher(Task, side_b_topic_name, 10)
        self._timer = self.create_timer(period, self._publish_task)

        self.get_logger().info(
            f'Publishing task on {topic_name} for scenario={scenario}, stage={stage} every {period:.2f}s '
            f'(publish_once={publish_once})'
        )
        self.get_logger().info(f'Publishing side-specific task for side_a on {side_a_topic_name}')
        self.get_logger().info(f'Publishing side-specific task for side_b on {side_b_topic_name}')

    def _publish_task(self) -> None:
        task = self._build_task()
        side_a_task = build_side_only_task(task, 'side_a')
        side_b_task = build_side_only_task(task, 'side_b')

        self._publisher.publish(task)
        self._side_a_publisher.publish(side_a_task)
        self._side_b_publisher.publish(side_b_task)

        self.get_logger().debug(
            f'Published task with {len(task.order_list)} orders and {len(task.arena_layout)} stations'
        )
        self.get_logger().debug(
            f'Published side_a task with {len(side_a_task.order_list)} orders and '
            f'{len(side_a_task.arena_layout)} stations'
        )
        self.get_logger().debug(
            f'Published side_b task with {len(side_b_task.order_list)} orders and '
            f'{len(side_b_task.arena_layout)} stations'
        )

        if self._publish_once:
            self.get_logger().info('Published one task. Shutting down.')
            self._timer.cancel()
            self._shutdown_requested = True


def main(args=None) -> None:
    rclpy.init(args=args)
    node = TaskPublisherNode()

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
