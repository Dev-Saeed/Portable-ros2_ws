import os
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, LaunchConfiguration

from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():

    send_test = Node(
        package="my_cpp_pkg",
        executable="hello_publisher_node"

    )

    recive_test = Node(
        package="my_cpp_pkg",
        executable="hello_subscriber_node"
        
    )

    return LaunchDescription([
        send_test,
        recive_test
    ])
