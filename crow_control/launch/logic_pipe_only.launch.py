from launch import LaunchDescription
import launch_ros.actions
from launch.actions import LogInfo, IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import ThisLaunchFileDir, LaunchConfiguration, PathJoinSubstitution
import os
import rclpy
from ros2topic import api
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    launchConfigs = []

    launchConfigs.append(
        launch_ros.actions.Node(
            package='crow_ontology',
            node_executable='server',
            output="screen"
        )
    )

    launchConfigs.append(
        launch_ros.actions.Node(
            package='crow_control',
            node_executable='logic',
            output="screen"
        )
    )

    return LaunchDescription(launchConfigs)
