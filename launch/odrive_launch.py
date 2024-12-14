from launch import LaunchDescription
from launch_ros.actions import Node

def get_launch_description():
    return [
        Node(package="magnetic_brake", executable="magnetic_brake_node"),
        Node(package="odrive_controller", executable="odrive_controller_node")
    ]