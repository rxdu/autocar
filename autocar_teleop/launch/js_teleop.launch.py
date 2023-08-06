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
    ## arguments

    ## local variables
    map_path = PathJoinSubstitution([
        FindPackageShare('robot_map'),
        "map",
        "my_office"
    ])

    rviz_config_file = PathJoinSubstitution([
        FindPackageShare('robot_bringup'),
        "rviz",
        "autoware_default_view.rviz"
    ])

    ## actions
    # launch_py_launch = IncludeLaunchDescription(
    #     PythonLaunchDescriptionSource([
    #         PathJoinSubstitution([
    #             FindPackageShare('autocar_teleop'),
    #             "launch",
    #             "module_launch.launch.py"
    #         ])
    #     ]),
    #     launch_arguments={
    #         'start_rviz2': 'false'
    #     }.items())

    # launch_xml_launch = IncludeLaunchDescription(
    #     FrontendLaunchDescriptionSource([
    #         PathJoinSubstitution([
    #             FindPackageShare('robot_nav'),
    #             "launch",
    #             'subsystem',
    #             "map.launch.xml"
    #         ])
    #     ]),
    #     launch_arguments={
    #         'map_path': map_path,
    #     }.items())
    #

    start_js_teleop_node = Node(
        package='autocar_teleop',
        executable='js_teleop',
        name='autocar_js_teleop',
        # arguments=['-d', rviz_config_file],
        output='screen')

    # start_rviz_node = Node(
    #     package='rviz2',
    #     executable='rviz2',
    #     name='rviz2_sample',
    #     arguments=['-d', rviz_config_file],
    #     output='screen')

    ## LaunchDescription
    return LaunchDescription([
        # launch_py_launch
        # launch_xml_launch,
        # start_rviz_node
        start_js_teleop_node
    ])
