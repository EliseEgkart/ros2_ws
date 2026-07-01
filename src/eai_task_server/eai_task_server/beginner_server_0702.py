#!/usr/bin/env python3

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


def make_station(station_type: int, name: str, station_id: int, material_ids: list[int]) -> Station:
    station = Station()
    station.station_type = station_type
    station.name = name
    station.station_id = station_id
    station.material_ids = material_ids
    return station


def build_production_beginner_task() -> Task:
    task = Task()
    task.order_list = [
        make_order(Order.OT_PRODUCE, 'produce_estop', 81),
        make_order(Order.OT_PRODUCE, 'produce_carrot', 442),
    ]
    task.arena_layout = [
        make_station(Station.ST_STORAGE, 'side_a_storage_1', 1, [2, 1]),
        make_station(Station.ST_STORAGE, 'side_a_storage_2', 2, [8]),
        make_station(Station.ST_STORAGE, 'side_a_storage_3', 3, [40]),
        make_station(Station.ST_WORKBENCH, 'side_a_workbench_1', 4, []),
        make_station(Station.ST_WORKBENCH, 'side_a_workbench_2', 5, []),
        make_station(Station.ST_HYBRID, 'side_a_hybrid_1', 6, []),
        make_station(Station.ST_CUSTOMER, 'side_a_customer_1', 7, []),
        make_station(Station.ST_STORAGE, 'side_b_storage_1', 8, [2, 1]),
        make_station(Station.ST_STORAGE, 'side_b_storage_2', 9, [8]),
        make_station(Station.ST_STORAGE, 'side_b_storage_3', 10, [40]),
        make_station(Station.ST_WORKBENCH, 'side_b_workbench_1', 11, []),
        make_station(Station.ST_WORKBENCH, 'side_b_workbench_2', 12, []),
        make_station(Station.ST_HYBRID, 'side_b_hybrid_1', 13, []),
        make_station(Station.ST_CUSTOMER, 'side_b_customer_1', 14, []),
    ]
    return task


def build_side_only_task(task: Task, side: str) -> Task:
    side_prefix = f'{side}_'
    side_task = Task()
    side_task.order_list = list(task.order_list)
    side_task.arena_layout = [
        station for station in task.arena_layout if station.name.startswith(side_prefix)
    ]
    return side_task


class BeginnerServer0702Node(Node):
    def __init__(self) -> None:
        super().__init__('beginner_server_0702')

        self.declare_parameter('topic_name', '/eai/task')
        self.declare_parameter('side_a_topic_name', '/eai/task/side_a')
        self.declare_parameter('side_b_topic_name', '/eai/task/side_b')
        self.declare_parameter('publish_period_sec', 1.0)
        self.declare_parameter('publish_once', False)

        topic_name = self.get_parameter('topic_name').get_parameter_value().string_value
        side_a_topic_name = self.get_parameter('side_a_topic_name').get_parameter_value().string_value
        side_b_topic_name = self.get_parameter('side_b_topic_name').get_parameter_value().string_value
        period = self.get_parameter('publish_period_sec').get_parameter_value().double_value
        self._publish_once = self.get_parameter('publish_once').get_parameter_value().bool_value
        self._shutdown_requested = False

        self._publisher = self.create_publisher(Task, topic_name, 10)
        self._side_a_publisher = self.create_publisher(Task, side_a_topic_name, 10)
        self._side_b_publisher = self.create_publisher(Task, side_b_topic_name, 10)
        self._timer = self.create_timer(period, self._publish_task)

        self.get_logger().info(
            f'Publishing fixed production beginner task on {topic_name} every {period:.2f}s '
            f'(publish_once={self._publish_once})'
        )
        self.get_logger().info(f'Publishing side-specific task for side_a on {side_a_topic_name}')
        self.get_logger().info(f'Publishing side-specific task for side_b on {side_b_topic_name}')

    def _publish_task(self) -> None:
        task = build_production_beginner_task()
        side_a_task = build_side_only_task(task, 'side_a')
        side_b_task = build_side_only_task(task, 'side_b')

        self._publisher.publish(task)
        self._side_a_publisher.publish(side_a_task)
        self._side_b_publisher.publish(side_b_task)

        self.get_logger().debug(
            f'Published fixed task with {len(task.order_list)} orders and '
            f'{len(task.arena_layout)} stations'
        )

        if self._publish_once:
            self.get_logger().info('Published one task. Shutting down.')
            self._timer.cancel()
            self._shutdown_requested = True


def main(args=None) -> None:
    rclpy.init(args=args)
    node = BeginnerServer0702Node()

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
