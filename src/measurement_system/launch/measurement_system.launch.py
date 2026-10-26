import os
import sys
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("ui_python", default_value=os.environ.get("DIPLOMKA_UI_PYTHON", sys.executable)),
        ExecuteProcess(cmd=[LaunchConfiguration("ui_python"), "-m", "measurement_system.ui"],
                       name="measurement_ui", output="screen"),
    ])