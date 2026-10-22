"""One command to start the ROS2 measurement nodes and the PySide6 UI.

The robot must not move during startup. Controller must remain IDLE until
/start_measurement is requested from the UI.
"""
import os
import sys

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, LogInfo
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    ui_python = LaunchConfiguration('ui_python')
    start_leica = LaunchConfiguration('start_leica')

    return LaunchDescription([
        DeclareLaunchArgument(
            'ui_python',
            default_value=os.environ.get('DIPLOMKA_UI_PYTHON', sys.executable),
            description='Python executable with PySide6, rclpy and ROS2 interfaces available',
        ),
        DeclareLaunchArgument(
            'start_leica',
            default_value='true',
            description='Launch /usr/local/bin/leica-driver (requires preconfigured permissions; no sudo)',
        ),
        Node(
            package='measurement_system',
            executable='robot_node',
            name='robot_node',
            output='screen',
        ),
        Node(
            package='measurement_system',
            executable='database_node',
            name='database_node',
            output='screen',
        ),
        Node(
            package='measurement_system',
            executable='controller_node',
            name='measurement_controller',
            output='screen',
        ),
        ExecuteProcess(
            cmd=[ui_python, '-m', 'measurement_system.ui'],
            name='measurement_ui',
            output='screen',
        ),
        ExecuteProcess(
            cmd=['/usr/local/bin/leica-driver'],
            name='leica_driver',
            output='screen',
            condition=IfCondition(start_leica),
        ),
        LogInfo(msg='Measurement system started. Measurement begins only on explicit UI start.'),
    ])