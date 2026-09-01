from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch.actions import RegisterEventHandler
from launch.actions import EmitEvent
from launch.events import Shutdown
from launch.event_handlers import OnProcessExit

from launch_ros.actions import Node


def generate_launch_description():

    controller = ExecuteProcess(
        cmd=[
            'ros2',
            'run',
            'measurement_system',
            'controller_node'
        ],
        output='screen'
    )

    robot = Node(
        package='measurement_system',
        executable='robot_node',
        name='robot_node',
        output='screen'
    )

    database = Node(
        package='measurement_system',
        executable='database_node',
        name='database_node',
        output='screen'
    )

    leica = ExecuteProcess(
        cmd=[
            'sudo',
            '/usr/local/bin/leica-driver'
        ],
        output='screen'
    )

    db_setup = ExecuteProcess(
        cmd=[
                    'python3',
                    '/home/dohnal/diplomka_ros/src/measurement_system/measurement_system/drivers/db_setup/databaze.py'
                ],
                output='screen'
            )

    shutdown_on_controller_exit = RegisterEventHandler(
        OnProcessExit(
            target_action=controller,
            on_exit=[
                EmitEvent(
                    event=Shutdown(
                        reason='Measurement sequence completed'
                    )
                )
            ]
        )
    )

    return LaunchDescription([
        robot,
        database,
        leica,
        controller,
        shutdown_on_controller_exit,
        db_setup
    ])