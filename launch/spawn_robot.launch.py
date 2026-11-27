"""Spawn one TurtleBot3 (waffle) into the running Gazebo world, with its Gazebo <-> ROS 2 bridge.

    ros2 launch launch/spawn_robot.launch.py namespace:=robot1 robot_name:=robot1 \\
        x_pose:=-4.8 y_pose:=-0.65 yaw:=0.0 bridge_clock:=true

The same as nav2_minimal_tb3_sim's spawn_tb3.launch.py (same robot model and topics), except
that the bridge config (config/tb3_bridge.yaml) has no /clock. Exactly one robot is spawned
with bridge_clock:=true, which starts the single /clock bridge: one clock bridge per robot made
two clocks interleave, and Nav2 saw time jump back and cleared its TF buffers mid-run.
"""
import os
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import AppendEnvironmentVariable, DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import Command, FindExecutable, LaunchConfiguration
from launch_ros.actions import Node

HERE = Path(__file__).resolve().parent


def generate_launch_description():
    sim = get_package_share_directory("nav2_minimal_tb3_sim")
    namespace = LaunchConfiguration("namespace")
    pose = {k: LaunchConfiguration(f"{k}_pose") for k in ("x", "y", "z")}
    return LaunchDescription([
        DeclareLaunchArgument("namespace", description="robot namespace, e.g. robot1"),
        DeclareLaunchArgument("robot_name", description="Gazebo model name (= robot id in the scenario)"),
        DeclareLaunchArgument("x_pose", default_value="0.0"),
        DeclareLaunchArgument("y_pose", default_value="0.0"),
        DeclareLaunchArgument("z_pose", default_value="0.01"),
        DeclareLaunchArgument("yaw", default_value="0.0"),
        DeclareLaunchArgument("bridge_clock", default_value="false",
                              description="also bridge /clock (true for exactly one robot)"),
        DeclareLaunchArgument("robot_sdf", default_value=os.path.join(sim, "urdf", "gz_waffle.sdf.xacro")),
        AppendEnvironmentVariable("GZ_SIM_RESOURCE_PATH", os.path.join(sim, "models")),
        AppendEnvironmentVariable("GZ_SIM_RESOURCE_PATH", str(Path(sim).parent.resolve())),
        Node(package="ros_gz_bridge", executable="parameter_bridge", namespace=namespace, output="screen",
             parameters=[{"config_file": str(HERE.parent / "config" / "tb3_bridge.yaml"),
                          "expand_gz_topic_names": True, "use_sim_time": True}]),
        Node(package="ros_gz_bridge", executable="parameter_bridge", name="clock_bridge", output="screen",
             arguments=["/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock"],
             condition=IfCondition(LaunchConfiguration("bridge_clock"))),
        Node(package="ros_gz_sim", executable="create", namespace=namespace, output="screen",
             arguments=["-name", LaunchConfiguration("robot_name"),
                        "-string", Command([FindExecutable(name="xacro"), " ", "namespace:=", namespace,
                                            " ", LaunchConfiguration("robot_sdf")]),
                        "-x", pose["x"], "-y", pose["y"], "-z", pose["z"],
                        "-R", "0.0", "-P", "0.0", "-Y", LaunchConfiguration("yaw")]),
    ])
