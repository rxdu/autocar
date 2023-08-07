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
    declear_joystick_index = DeclareLaunchArgument(
        'joystick_index',
        default_value='0',
        description='Joystick index')

    declear_vesc_can_if_name = DeclareLaunchArgument(
        'vesc_can_if_name',
        default_value='can0',
        description='VESC CAN interface name')

    declear_vesc_id = DeclareLaunchArgument(
        "vesc_id",
        default_value="0x68",
        description="VESC ID")

    declear_cmd_topic_name = DeclareLaunchArgument(
        "cmd_topic_name",
        default_value="/cmd_vel",
        description="Command topic name")

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

    # start_rviz_node = Node(
    #     package='rviz2',
    #     executable='rviz2',
    #     name='rviz2_sample',
    #     arguments=['-d', rviz_config_file],
    #     output='screen')

    start_js_teleop_node = Node(
        package='autocar_teleop',
        executable='js_teleop',
        name='autocar_js_teleop',
        # arguments=['-d', rviz_config_file],
        output='screen',
        parameters=[
            {"joystick_index": LaunchConfiguration("joystick_index"),
             "vesc_can_if_name": LaunchConfiguration("vesc_can_if_name"),
             "vesc_id": LaunchConfiguration("vesc_id"),
             "cmd_topic_name": LaunchConfiguration("cmd_topic_name")}
        ])

    ## LaunchDescription
    return LaunchDescription([
        declear_joystick_index,
        declear_vesc_can_if_name,
        declear_vesc_id,
        declear_cmd_topic_name,
        # launch_py_launch
        # launch_xml_launch,
        # start_rviz_node
        start_js_teleop_node
    ])
