from __future__ import annotations

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    pkg_share = get_package_share_directory("physicai_arm_gz")
    ros_gz_sim_share = get_package_share_directory("ros_gz_sim")

    world_arg = DeclareLaunchArgument(
        "world_sdf",
        description="Absolute path to your static_world.sdf file.",
    )

    use_rsp_arg = DeclareLaunchArgument(
        "use_robot_state_publisher",
        default_value="true",
        description="Whether to run robot_state_publisher for TF publication.",
    )

    auto_run_arg = DeclareLaunchArgument(
        "auto_run",
        default_value="true",
        description="Pass -r to Gazebo so physics starts immediately.",
    )

    model_search_path = os.path.join(pkg_share, "models")
    existing = os.environ.get("GZ_SIM_RESOURCE_PATH", "")
    combined = model_search_path if not existing else f"{model_search_path}:{existing}"

    gz_args = PythonExpression([
        '"',
        LaunchConfiguration("world_sdf"),
        '" + (" -r" if "',
        LaunchConfiguration("auto_run"),
        '" == "true" else "")',
    ])

    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(ros_gz_sim_share, "launch", "gz_sim.launch.py")
        ),
        launch_arguments={"gz_args": gz_args}.items(),
    )

    bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        output="screen",
        parameters=[{"config_file": os.path.join(pkg_share, "config", "bridge.yaml")}],
    )

    adapter = Node(
        package="physicai_arm_gz",
        executable="joint_targets_to_cmd_pos",
        output="screen",
    )

    robot_description_path = os.path.join(pkg_share, "urdf", "so101_new_calib.urdf")
    with open(robot_description_path, "r", encoding="utf-8") as f:
        robot_description = f.read()

    rsp = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[{"robot_description": robot_description}],
        condition=IfCondition(LaunchConfiguration("use_robot_state_publisher")),
    )

    return LaunchDescription(
        [
            world_arg,
            use_rsp_arg,
            auto_run_arg,
            SetEnvironmentVariable("GZ_SIM_RESOURCE_PATH", combined),
            gz_sim,
            bridge,
            adapter,
            rsp,
        ]
    )
