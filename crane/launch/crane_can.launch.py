"""
Vinc sahasi + CAN vinc ECU'su (can_bridge) tek komutla. Once: ./crane/can/setup_vcan.sh

Kullanim (proje klasorunden):
    ros2 launch crane/launch/crane_can.launch.py
    python3 crane/voice_crane.py            # ikinci terminal: sesli kumanda (ya da --text)
"""
import os

from launch import LaunchDescription
from launch.actions import ExecuteProcess, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

CRANE_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))


def generate_launch_description():
    return LaunchDescription([
        IncludeLaunchDescription(PythonLaunchDescriptionSource(
            os.path.join(CRANE_DIR, "launch", "crane.launch.py"))),
        ExecuteProcess(
            cmd=["python3", os.path.join(CRANE_DIR, "can", "can_bridge.py")],
            cwd=CRANE_DIR,
            output="screen"),
    ])
