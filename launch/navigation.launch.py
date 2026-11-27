"""Nav2 for every robot in the scenario, each in its own namespace (robot1, robot2, ...).

    ros2 launch launch/navigation.launch.py scenario:=config/scenario.yaml

Start it after Gazebo and the robots are up (scripts/run.sh --nav does this). For each robot:
  - robot_state_publisher: the TurtleBot3 body frames (base_footprint -> base_link -> base_scan ...)
  - a fixed transform map -> odom = the robot's start pose (no AMCL; see config/nav2_params.yaml)
  - controller_server, planner_server, behavior_server, bt_navigator and a lifecycle manager
    that starts them. The robot then accepts goals on /<robot>/navigate_to_pose.

Each robot has its own TF tree on /<robot>/tf and /<robot>/tf_static. The Gazebo bridge started
by spawn_tb3.launch.py already provides /<robot>/odom, /<robot>/scan, /<robot>/tf (odom ->
base_footprint), /<robot>/joint_states and /<robot>/cmd_vel.
"""
import math
import tempfile
from pathlib import Path

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

HERE = Path(__file__).resolve().parent
NAV_NODES = [("controller_server", "nav2_controller"), ("planner_server", "nav2_planner"),
             ("behavior_server", "nav2_behaviors"), ("bt_navigator", "nav2_bt_navigator")]
MAP_MARGIN_M = 1.0     # global costmap reaches this far beyond the outer room walls


def global_costmap_window(rooms):
    """origin_x, origin_y, width, height (m) of a fixed window that covers every room."""
    x0 = min(r["bounds"][0] for r in rooms) - MAP_MARGIN_M
    y0 = min(r["bounds"][1] for r in rooms) - MAP_MARGIN_M
    x1 = max(r["bounds"][2] for r in rooms) + MAP_MARGIN_M
    y1 = max(r["bounds"][3] for r in rooms) + MAP_MARGIN_M
    return x0, y0, int(math.ceil(x1 - x0)), int(math.ceil(y1 - y0))


def setup(context):
    scenario = Path(LaunchConfiguration("scenario").perform(context)).resolve()
    params_file = Path(LaunchConfiguration("params_file").perform(context)).resolve()
    sc = yaml.safe_load(scenario.read_text())
    params = yaml.safe_load(params_file.read_text())

    x0, y0, width, height = global_costmap_window(sc["rooms"])
    params["global_costmap"]["global_costmap"]["ros__parameters"].update(
        origin_x=float(x0), origin_y=float(y0), width=width, height=height)

    sim = Path(get_package_share_directory("nav2_minimal_tb3_sim"))
    urdf = (sim / "urdf" / "turtlebot3_waffle.urdf").read_text()
    out_dir = Path(tempfile.mkdtemp(prefix="escape_maze_nav2_"))
    remaps = [("/tf", "tf"), ("/tf_static", "tf_static")]
    sim_time = {"use_sim_time": True}

    actions = []
    for robot in sc["robots"]:
        rid = robot["id"]
        x, y, yaw = (list(robot["start_pose"]) + [0.0, 0.0, 0.0])[:3]
        robot_params = out_dir / f"{rid}_nav2.yaml"
        robot_params.write_text(yaml.safe_dump({rid: params}))   # parameters under the namespace

        actions += [
            Node(package="robot_state_publisher", executable="robot_state_publisher", namespace=rid,
                 parameters=[{**sim_time, "robot_description": urdf}], remappings=remaps, output="log"),
            Node(package="tf2_ros", executable="static_transform_publisher", namespace=rid,
                 name="map_to_odom",
                 arguments=["--x", str(x), "--y", str(y), "--yaw", str(yaw),
                            "--frame-id", "map", "--child-frame-id", "odom"],
                 parameters=[sim_time], remappings=remaps, output="log"),
        ]
        for name, package in NAV_NODES:
            actions.append(Node(package=package, executable=name, name=name, namespace=rid,
                                parameters=[str(robot_params), sim_time], remappings=remaps,
                                output="log"))
        actions.append(Node(
            package="nav2_lifecycle_manager", executable="lifecycle_manager",
            name="lifecycle_manager_navigation", namespace=rid, output="screen",
            parameters=[{**sim_time, "autostart": True, "bond_timeout": 10.0,
                         "node_names": [name for name, _ in NAV_NODES]}]))
    return actions


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("scenario", default_value=str(HERE.parent / "config" / "scenario.yaml")),
        DeclareLaunchArgument("params_file", default_value=str(HERE.parent / "config" / "nav2_params.yaml")),
        OpaqueFunction(function=setup),
    ])
