import os
from pathlib import Path
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
#from launch.conditions import IfCondition
#from launch.conditions import UnlessCondition
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
#from launch.launch_description_sources import PythonLaunchDescriptionSource
#from launch.substitutions import IncludeLaunchDescription
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
#from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node

def generate_launch_description():

    return LaunchDescription([

            Node(
                package='rf2o_laser_odometry',
                executable='rf2o_laser_odometry_node',
                name='rf2o_laser_odometry',
                output='screen',
                parameters=[{
                    'laser_scan_topic' : '/scan',
                    'odom_topic' : '/odom',
                    'publish_tf' : True,
                    'base_frame_id' : 'base_footprint',
                    'odom_frame_id' : 'odom',
                    'init_pose_from_topic' : '',
                    'freq' : 20.0
                }],
            ),

            IncludeLaunchDescription(
                PathJoinSubstitution([
                    FindPackageShare('slam_toolbox'),
                    'launch','online_async_launch.py'
                ]),
                launch_arguments={
                    'autostart' : 'true',
                    'use_lifecycle_manager'  : 'false',
                    'use_sim_time' : 'true',
                    'slam_params_file' : './slam_toolbox.yaml',
                }.items()
            ),


            Node(
                package='tf2_ros',
                executable='static_transform_publisher',
                name='base_link_to_base_tof',
                arguments=[
                    "--x", "0.0",
                    "--y", "0.0",
                    "--z", "0.0",
                    "--roll", "0.0",
                    "--pitch", "0.0",
                    "--yaw", "0.0",
                    "--frame-id", "map",
                    "--child-frame-id", "odom"],
                output="screen"
            ),

            Node(
                package='tf2_ros',
                executable='static_transform_publisher',
                name='base_link_to_base_tof',
                arguments=[
                    "--x", "0.0",
                    "--y", "0.0",
                    "--z", "0.0",
                    "--roll", "0.0",
                    "--pitch", "0.0",
                    "--yaw", "0.0",
                    "--frame-id", "odom",
                    "--child-frame-id", "base_footprint"],
                output="screen"
            ),

            Node(
                package='tf2_ros',
                executable='static_transform_publisher',
                name='base_link_to_base_tof',
                arguments=[
                    "--x", "0.0",
                    "--y", "0.0",
                    "--z", "0.0",
                    "--roll", "0.0",
                    "--pitch", "0.0",
                    "--yaw", "0.0",
                    "--frame-id", "base_footprint",
                    "--child-frame-id", "base_link"],
                output="screen"
            ),

            # RPLidar TF
            Node(
                package='tf2_ros',
                executable='static_transform_publisher',
                name='base_link_to_base_tof',
                arguments=[
                    "--x", "0.0",
                    "--y", "0.0",
                    "--z", "0.0",
                    "--roll", "0.0",
                    "--pitch", "0.0",
                    "--yaw", "0.0",
                    "--frame-id", "base_link",
                    "--child-frame-id", "laser"],
                output="screen"
            )
    ])
