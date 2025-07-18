import os
import yaml

from launch import LaunchDescription
from launch_ros.actions import Node

from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

from launch.actions import GroupAction
from launch_ros.actions import PushRosNamespace

from launch.actions import ExecuteProcess
from launch.substitutions import EnvironmentVariable, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():

    robot_namespace = LaunchConfiguration('robot_namespace')

    bento_drive = Node(
        package='bento_drive',
        executable='bento_drive_node',
        name='bento_drive_node',
        parameters=[ PathJoinSubstitution([ '/', 'launch-content', 'parameters', 'bento-box.yaml' ]) ],
        remappings=[('odom', '/odom' ), ('pose', '/pose')],
        output='screen',
	emulate_tty=True,
    )

    # There is some bug in bento_drive,
    # where the motorcontrollers de-enable without an eduart power-management board sending something.
    # Bento-Box has no such board, so we just send the 'important' thing the board would send
    # (I think it is a voltage readout, because bento_drive complains about undervoltage)
    can_fix = ExecuteProcess(
        cmd=[['while true; do cansend can0 580#0241AD347300; sleep 1; done']],
        shell=True,
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

    lidar = Node(
        name='rplidar_composition',
        package='rplidar_ros',
        executable='rplidar_composition',
        output='screen',
        emulate_tty=True,
        parameters=[{
            'serial_port': '/dev/ttyUSB0',
            'serial_baudrate': 115200,  # A1 / A2
            # 'serial_baudrate': 256000, # A3
            'frame_id': 'laser',
            'inverted': False,
            'angle_compensate': True,
        }],
    )

    tf2_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='base_link_to_base_tof',
        arguments=["--x", "0.0","--y", "0.0","--z", "0.1","--roll", "0.0","--pitch", "0.0","--yaw", "0.0",
        #TODO add values
        "--frame-id", "base_link","--child-frame-id", "laser"],
        output="screen"
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
            #lidar,
        ]),
        lidar,
        tf2_node,
        can_fix,
    ])
