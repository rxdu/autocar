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
        default_value='19',
        description='Joystick index')

    declear_vesc_can_if_name = DeclareLaunchArgument(
        'vesc_can_if_name',
        default_value='vcan0',
        description='VESC CAN interface name')

    declear_vesc_id = DeclareLaunchArgument(
        "vesc_id",
        default_value="0x68",
        description="VESC ID")

    declear_cmd_topic_name = DeclareLaunchArgument(
        "cmd_topic_name",
        default_value="/cmd_vel",
        description="Command topic name")

    declear_neutral_steer_angle = DeclareLaunchArgument(
        "neutral_steer_angle",
        default_value="0.5",
        description="Neutral steer angle")

    declare_max_steer_angle = DeclareLaunchArgument(
        "max_steer_angle",
        default_value="1.0",
        description="Max steer angle")

    declare_min_steer_angle = DeclareLaunchArgument(
        "min_steer_angle",
        default_value="0.0",
        description="Min steer angle")

    declare_steer_angle_deadzone = DeclareLaunchArgument(
        "steer_angle_deadzone",
        default_value="0.05",
        description="Steer angle deadzone")

    declare_max_motor_rpm = DeclareLaunchArgument(
        "max_motor_rpm",
        default_value="3000",
        description="Max motor rpm")

    declare_min_motor_rpm = DeclareLaunchArgument(
        "min_motor_rpm",
        default_value="-2000",
        description="Min motor rpm")

    declare_motor_rpm_deadzone = DeclareLaunchArgument(
        "motor_rpm_deadzone",
        default_value="50",
        description="Motor rpm deadzone")

    declare_rpm_ratio = DeclareLaunchArgument(
        "rpm_ratio",
        default_value="1000.0",
        description="RPM ratio")

    declare_steer_ratio = DeclareLaunchArgument(
        "steer_ratio",
        default_value="1.0",
        description="Steer ratio")

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
        output='screen',
        parameters=[
            {"joystick_index": LaunchConfiguration("joystick_index"),
             "vesc_can_if_name": LaunchConfiguration("vesc_can_if_name"),
             "vesc_id": LaunchConfiguration("vesc_id"),
             "cmd_topic_name": LaunchConfiguration("cmd_topic_name"),
             "neutral_steer_angle": LaunchConfiguration("neutral_steer_angle"),
             "steer_angle_deadzone": LaunchConfiguration("steer_angle_deadzone"),
             "max_steer_angle": LaunchConfiguration("max_steer_angle"),
             "min_steer_angle": LaunchConfiguration("min_steer_angle"),
             "max_motor_rpm": LaunchConfiguration("max_motor_rpm"),
             "min_motor_rpm": LaunchConfiguration("min_motor_rpm"),
             "motor_rpm_deadzone": LaunchConfiguration("motor_rpm_deadzone"),
             "rpm_ratio": LaunchConfiguration("rpm_ratio"),
             "steer_ratio": LaunchConfiguration("steer_ratio")}
        ])

    ## LaunchDescription
    return LaunchDescription([
        declear_joystick_index,
        declear_vesc_can_if_name,
        declear_vesc_id,
        declear_cmd_topic_name,
        declear_neutral_steer_angle,
        declare_max_steer_angle,
        declare_min_steer_angle,
        declare_steer_angle_deadzone,
        declare_max_motor_rpm,
        declare_min_motor_rpm,
        declare_motor_rpm_deadzone,
        declare_rpm_ratio,
        declare_steer_ratio,
        # start_rviz_node
        start_js_teleop_node
    ])
