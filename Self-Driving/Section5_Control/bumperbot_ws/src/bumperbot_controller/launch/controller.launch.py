import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import RegisterEventHandler
from launch.event_handlers import OnProcessExit


def generate_launch_description():

    # Spawner for the joint_state_broadcaster
    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "joint_state_broadcaster",
            "--controller-manager",
            "/controller_manager",
        ],
    )

    # Spawner for the simple_velocity_controller (spawned after joint_state_broadcaster_spawner exits)
    simple_controller = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["simple_velocity_controller",
                   "--controller-manager",
                   "/controller_manager"
        ]
    )

    # EventHandler to trigger simple_velocity_controller after joint_state_broadcaster_spawner exits
    event_handler = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_state_broadcaster_spawner,  # Target action to wait for
            on_exit=[simple_controller]  # Action to trigger after exit
        )
    )

    return LaunchDescription(
        [
            joint_state_broadcaster_spawner,  # Launch joint_state_broadcaster_spawner first
            event_handler,                    # Event handler to start simple_controller after joint_state_broadcaster exits
        ]
    )
