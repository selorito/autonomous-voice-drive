"""
Demo: Gazebo (isaret kutulu sahne) + TurtleBot3 + motion_node tek komutla.
voice_node klavyeden Enter beklediği icin AYRI terminalde calistirilir.

Kullanim (proje klasorunden):
    ros2 launch launch/demo.launch.py
    python3 voice_node.py          # ikinci terminal
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess, IncludeLaunchDescription, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

PROJECT_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
WORLD = os.path.join(PROJECT_DIR, "worlds", "demo.world")


def generate_launch_description():
    gazebo_ros = get_package_share_directory("gazebo_ros")
    tb3_launch = os.path.join(get_package_share_directory("turtlebot3_gazebo"), "launch")
    use_sim_time = LaunchConfiguration("use_sim_time", default="true")

    return LaunchDescription([
        SetEnvironmentVariable("TURTLEBOT3_MODEL", "burger"),
        SetEnvironmentVariable("GAZEBO_MODEL_DATABASE_URI", ""),   # internetten model indirmeye calismasin

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(gazebo_ros, "launch", "gzserver.launch.py")),
            launch_arguments={"world": WORLD}.items()),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(gazebo_ros, "launch", "gzclient.launch.py"))),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(tb3_launch, "robot_state_publisher.launch.py")),
            launch_arguments={"use_sim_time": use_sim_time}.items()),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(tb3_launch, "spawn_turtlebot3.launch.py")),
            launch_arguments={"x_pose": "0.0", "y_pose": "0.0"}.items()),

        ExecuteProcess(
            cmd=["python3", os.path.join(PROJECT_DIR, "motion_node.py")],
            output="screen"),
    ])
