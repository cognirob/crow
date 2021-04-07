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
        )
    )

    # cams & detection
    cam_launcher = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([PathJoinSubstitution([get_package_share_directory("crow_vision_ros2"), "launch"]), os.path.sep, "full_crow_object.launch.py"]),
        launch_arguments={'camera_config': LaunchConfiguration('camera_config')}.items()
    )
    launchConfigs.append(LogInfo(msg="Launching image processing pipeline"))
    launchConfigs.append(cam_launcher)

    launchConfigs.append(
        launch_ros.actions.Node(
            package='crow_ontology',
            node_executable='adder',
            output="log"
        )
    )

    # launchConfigs.append(
    #     launch_ros.actions.Node(
    #         package='crow_control',
    #         node_executable='logic',
    #         output="screen"
    #     )
    # )


    return LaunchDescription([
        DeclareLaunchArgument("camera_config", default_value="rs_native.yaml")
    ] + launchConfigs)
