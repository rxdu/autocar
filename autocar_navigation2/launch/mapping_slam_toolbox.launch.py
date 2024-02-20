import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time',default='false')
    config_file_name = 'slam_toolbox_params.yaml'
    rviz_file_name = 'mapping.rviz'

    slam_toolbox_config = os.path.join(
        get_package_share_directory('autocar_navigation2'), 
        'config',
        config_file_name)
    
    slam_toolbox_rviz = os.path.join(
        get_package_share_directory('autocar_navigation2'),
        'rviz',
        rviz_file_name)

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation (Gazebo) clock if true'),
        Node(
            package='slam_toolbox',
            executable='sync_slam_toolbox_node',
            name='mapping_slam_toolbox',
            parameters=[slam_toolbox_config, {'use_sim_time': use_sim_time}],
            output='screen'),
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', slam_toolbox_rviz],
            output='screen')
    ])