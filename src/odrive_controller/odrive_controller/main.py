from contextlib import contextmanager

import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node

from .odrive_motor_controller import OdriveController


@contextmanager
def rclpy_init():
    rclpy.init()
    try:
        yield
    finally:
        rclpy.shutdown()


def main():
    with rclpy_init:
        rclpy.init()
        node = Node("odrive_ros2")
        executor = SingleThreadedExecutor()
        executor.add_node(node)
        OdriveController(node)
        try:
            executor.spin()
        finally:
            executor.remove_node(node)
            executor.shutdown()
            node.destroy_node()


if __name__ == "__main__":
    main()
