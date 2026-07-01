#!/usr/bin/env python3

import random

import rclpy
from eai_task_server.task_complexity_publisher import (
    COMPLEXITY,
    build_side_only_task,
    build_task_from_products,
)
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from sml_messages.msg import Task


def build_production_beginner_task(rng: random.Random) -> Task:
    rule = COMPLEXITY[('beginner', 'production')]
    return build_task_from_products(
        rule,
        produce_ids=[81, 442],
        recycle_ids=[],
        selected_sides=['side_a', 'side_b'],
        rng=rng,
    )


class BeginnerServer0702Node(Node):
    def __init__(self) -> None:
        super().__init__('beginner_server_0702')

        self.declare_parameter('topic_name', '/eai/task')
        self.declare_parameter('side_a_topic_name', '/eai/task/side_a')
        self.declare_parameter('side_b_topic_name', '/eai/task/side_b')
        self.declare_parameter('publish_period_sec', 1.0)
        self.declare_parameter('publish_once', True)
        self.declare_parameter('seed', 0)

        topic_name = self.get_parameter('topic_name').get_parameter_value().string_value
        side_a_topic_name = self.get_parameter('side_a_topic_name').get_parameter_value().string_value
        side_b_topic_name = self.get_parameter('side_b_topic_name').get_parameter_value().string_value
        period = self.get_parameter('publish_period_sec').get_parameter_value().double_value
        self._publish_once = self.get_parameter('publish_once').get_parameter_value().bool_value
        self._shutdown_requested = False
        seed = self.get_parameter('seed').get_parameter_value().integer_value
        self._rng = random.Random(seed if seed != 0 else None)
        self._task = build_production_beginner_task(self._rng)

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
