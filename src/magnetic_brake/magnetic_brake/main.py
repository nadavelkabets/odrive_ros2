from contextlib import contextmanager

import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node

from .magnetic_brake_handler import MagneticBrakeHandler


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
        MagneticBrakeHandler(node)
        try:
            executor.spin()
        finally:
            executor.remove_node(node)
            executor.shutdown()
            node.destroy_node()


if __name__ == "__main__":
    main()
