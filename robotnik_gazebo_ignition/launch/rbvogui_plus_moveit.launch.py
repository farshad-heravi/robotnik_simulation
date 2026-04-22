"""
Launch MoveIt2 for the UR5e arm on the rbvogui_plus rover.

Prerequisites (already running):
  ros2 launch robotnik_gazebo_ignition spawn_robot.launch.py \
      robot_id:=robot robot:=rbvogui robot_model:=rbvogui_plus

Usage:
  ros2 launch robotnik_gazebo_ignition rbvogui_plus_moveit.launch.py
"""

import os
import yaml
from pathlib import Path

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration

from launch_ros.actions import Node

from ament_index_python.packages import get_package_share_directory
from moveit_configs_utils import MoveItConfigsBuilder


def load_yaml(path):
    with open(path) as f:
        return yaml.safe_load(f)


def generate_launch_description():
    use_sim_time = LaunchConfiguration("use_sim_time")
    launch_rviz  = LaunchConfiguration("launch_rviz")

    declared_arguments = [
        DeclareLaunchArgument("use_sim_time", default_value="true",
                              description="Use simulation (Gazebo) clock"),
        DeclareLaunchArgument("launch_rviz", default_value="true",
                              description="Launch RViz with MoveIt plugin"),
    ]

    pkg_gz     = get_package_share_directory("robotnik_gazebo_ignition")
    pkg_moveit = get_package_share_directory("ur_moveit_config")

    moveit_dir = os.path.join(pkg_gz, "config", "profile", "rbvogui_plus", "moveit")

    # ── Load our custom SRDF and controllers ────────────────────────────────────
    srdf_file        = os.path.join(moveit_dir, "rbvogui_plus_arm.srdf")
    controllers_file = os.path.join(moveit_dir, "moveit_controllers.yaml")

    with open(srdf_file) as f:
        srdf_content = f.read()

    controllers_yaml = load_yaml(controllers_file)

    # ── Use MoveItConfigsBuilder to get the full OMPL planner config ─────────
    # We use ur_moveit_config as the base to get all planner_configs generated,
    # but override the SRDF with ours (which has the robot_arm_ prefixed joints).
    moveit_config = (
        MoveItConfigsBuilder(robot_name="ur", package_name="ur_moveit_config")
        .robot_description_semantic(
            Path(pkg_moveit) / "srdf" / "ur.srdf.xacro",
            {"name": "ur5e"},
        )
        .planning_pipelines(pipelines=["ompl"])
        .to_moveit_configs()
    )

    # Override the SRDF with our custom one (robot_arm_ prefix)
    planning_pipelines = moveit_config.planning_pipelines

    kinematics_yaml   = load_yaml(os.path.join(pkg_moveit, "config", "kinematics.yaml"))
    joint_limits_yaml = load_yaml(os.path.join(pkg_moveit, "config", "joint_limits.yaml"))

    # ── move_group ─────────────────────────────────────────────────────────────
    move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        namespace="robot",
        parameters=[
            {
                "robot_description_semantic": srdf_content,
                "robot_description_kinematics": kinematics_yaml,
                "robot_description_planning": joint_limits_yaml,
                "use_sim_time": use_sim_time,
                "robot_description_topic": "robot_description",
                "moveit_simple_controller_manager.controller_manager_name":
                    "/robot/controller_manager",
            },
            planning_pipelines,
            controllers_yaml,
        ],
    )

    # ── RViz ───────────────────────────────────────────────────────────────────
    rviz_config = os.path.join(moveit_dir, "rbvogui_plus_moveit.rviz")
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2_moveit",
        namespace="robot",
        output="log",
        condition=IfCondition(launch_rviz),
        arguments=["-d", rviz_config],
        parameters=[
            {
                "robot_description_semantic": srdf_content,
                "use_sim_time": use_sim_time,
            }
        ],
    )

    return LaunchDescription(declared_arguments + [
        move_group_node,
        TimerAction(period=4.0, actions=[rviz_node]),
    ])
