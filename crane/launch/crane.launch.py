"""
MPG tarzi kamyon vinci sahasi: Gazebo + crane_controller tek komutla.

Kullanim (proje klasorunden):
    ros2 launch crane/launch/crane.launch.py               # kanca kamerasi penceresi ile
    ros2 launch crane/launch/crane.launch.py vision:=false # goruntu isleme kapali (zayif bilgisayar)
Hiz: CRANE_SPEED=1 ros2 launch ...   (varsayilan 2 = gercekcinin 2 kati, sunumda bekleme azalsin)
Test:
    ros2 topic pub --once /crane/command std_msgs/msg/String "{data: 'MOVE kirmizi_kasa depo'}"
    ros2 topic echo /crane/status
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription, SetEnvironmentVariable
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource

CRANE_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
WORLD = os.path.join(CRANE_DIR, "worlds", "crane_site.world")


def generate_launch_description():
    gazebo_ros = get_package_share_directory("gazebo_ros")
    return LaunchDescription([
        DeclareLaunchArgument("vision", default_value="true", description="kanca kamerasi goruntu isleme"),
        SetEnvironmentVariable("GAZEBO_MODEL_DATABASE_URI", ""),   # internetten model indirmeye calismasin

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(gazebo_ros, "launch", "gzserver.launch.py")),
            launch_arguments={"world": WORLD}.items()),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(gazebo_ros, "launch", "gzclient.launch.py"))),

        ExecuteProcess(
            cmd=["python3", os.path.join(CRANE_DIR, "crane_controller.py")],
            cwd=CRANE_DIR,
            output="screen"),
        ExecuteProcess(
            cmd=["python3", os.path.join(CRANE_DIR, "vision_node.py")],
            cwd=CRANE_DIR,
            output="screen",
            condition=IfCondition(LaunchConfiguration("vision"))),
    ])
