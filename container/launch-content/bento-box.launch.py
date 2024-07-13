import os
import yaml

from launch import LaunchDescription
from launch_ros.actions import Node

from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

from launch.actions import GroupAction
from launch_ros.actions import PushRosNamespace

from launch.substitutions import EnvironmentVariable, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():

    robot_namespace = LaunchConfiguration('robot_namespace')

    bento_drive = Node(
        package='bento_drive',
        executable='bento_drive_node',
        name='bento_drive_node',
        parameters=[ PathJoinSubstitution([ '/', 'launch-content', 'parameters', 'bento-box.yaml' ]) ],
        output='screen',
	emulate_tty=True,
    )

    joystick = Node(
        package='joy_linux',
        executable='joy_linux_node',
    )

    camera_ros_1 = Node(
        package='camera_ros',
        executable='camera_node',
        name='camera_node_1',
        parameters=[ PathJoinSubstitution([ '/', 'launch-content', 'parameters', 'camera_ros-1.yaml' ]) ],
        namespace="cam1",
        emulate_tty=True,
    )

    camera_ros_2 = Node(
        package='camera_ros',
        executable='camera_node',
        name='camera_node_2',
        parameters=[ PathJoinSubstitution([ '/', 'launch-content', 'parameters', 'camera_ros-2.yaml' ]) ],
        namespace="cam2",
        emulate_tty=True,
    )

    lidar = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('rplidar_ros'),
                'launch',
                'rplidar_a1_launch.py'
            ])
        ]),
        launch_arguments={
            'channel_type': 'serial',
            'serial_port': '/dev/ttyUSB0',
            'serial_baudrate': '115200',
            'frame_id': 'laser',
            'inverted': 'false',
            'angle_compensate': 'true',
            #'scan_mode': 'Sensitivity',
        }.items(),
    )


    return LaunchDescription([
        DeclareLaunchArgument(
            'robot_namespace',
            default_value='bento',
            description='set namespace for robot nodes'
        ),
        GroupAction(
        actions=[
            PushRosNamespace(robot_namespace),
            joystick,
            camera_ros_1,
            camera_ros_2,
            bento_drive,
            lidar,
        ])
    ])
