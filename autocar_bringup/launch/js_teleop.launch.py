from launch import LaunchDescription

from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node

from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.launch_description_sources import FrontendLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch.substitutions import PathJoinSubstitution
from launch.substitutions import (EnvironmentVariable, FindExecutable)
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    ## load params
    teleop_config=PathJoinSubstitution([
        FindPackageShare('autocar_bringup'),
        "params",
        "js_teleop.yaml"
    ])

    ## actions
    start_js_teleop_node = Node(
        package='autocar_teleop',
        executable='js_teleop',
        name='autocar_js_teleop',
        output='screen',
        # prefix=['xterm -e gdb -ex run --args'],
        parameters=[teleop_config])

    ## LaunchDescription
    return LaunchDescription([
        start_js_teleop_node
    ])
