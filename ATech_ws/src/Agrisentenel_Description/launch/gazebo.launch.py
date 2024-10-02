import os
from os import pathsep
from ament_index_python.packages import get_package_share_directory, get_package_prefix

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable
from launch.substitutions import Command, LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

# Function to generate the launch description for running the robot simulation
def generate_launch_description():
    
    # Get the path to the Agrisentenel_Description package, which contains the URDF and other resources
    Agrisentenel_Description = get_package_share_directory("Agrisentenel_Description")
    
    # Get the installation prefix of the Agrisentenel_Description package
    Agrisentenel_Description_prefix = get_package_prefix("Agrisentenel_Description")
    
    # Get the path to the gazebo_ros package, used to start the Gazebo simulator
    gazebo_ros_dir = get_package_share_directory("gazebo_ros")

    # Declare a launch argument that allows the user to specify the path to the robot model (URDF file)
    model_arg = DeclareLaunchArgument(name="model", 
                                      default_value=os.path.join(Agrisentenel_Description, "urdf", "agrisentenel.urdf.xacro"),
                                      description="Absolute path to robot urdf file"
    )

    # Set up the GAZEBO_MODEL_PATH environment variable to include paths to the robot models
    model_path = os.path.join(Agrisentenel_Description, "models")
    model_path += pathsep + os.path.join(Agrisentenel_Description_prefix, "share")

    # Set the GAZEBO_MODEL_PATH environment variable so Gazebo can locate the robot model files
    env_var = SetEnvironmentVariable("GAZEBO_MODEL_PATH", model_path)

    # Create the robot_description parameter using the xacro command to process the URDF file specified by the "model" argument
    robot_description = ParameterValue(Command(["xacro ", LaunchConfiguration("model")]),
                                       value_type=str)

    # Node to publish the robot state using the robot_state_publisher package, which reads the robot_description and broadcasts the joint states
    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[{"robot_description": robot_description}]
    )

    # Include the Gazebo server launch file to start the Gazebo simulation backend (physics and environment)
    start_gazebo_server = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(gazebo_ros_dir, "launch", "gzserver.launch.py")
        )
    )

    # Include the Gazebo client launch file to start the Gazebo graphical interface (visualizing the simulation)
    start_gazebo_client = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(gazebo_ros_dir, "launch", "gzclient.launch.py")
        )
    )

    # Node to spawn the robot in the Gazebo simulation using the spawn_entity.py script from the gazebo_ros package
    spawn_robot = Node(package="gazebo_ros", executable="spawn_entity.py",
                        arguments=["-entity", "agrisentenel",
                                   "-topic", "robot_description",
                                  ],
                        output="screen"
    )

    # Return the full launch description, which sets up the environment variables, launches Gazebo, publishes robot states, and spawns the robot
    return LaunchDescription([
        env_var,                  # Set GAZEBO_MODEL_PATH so Gazebo can find the robot model
        model_arg,                # Allow specifying the robot URDF file
        start_gazebo_server,      # Launch Gazebo server (physics engine)
        start_gazebo_client,      # Launch Gazebo client (GUI)
        robot_state_publisher_node, # Publish robot state based on URDF
        spawn_robot               # Spawn the robot into the Gazebo simulation
    ])
