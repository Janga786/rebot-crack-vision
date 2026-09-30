"""Headless MoveIt + ros2_control mock hardware bring-up for the B601-DM model (MOT-02).

Same node graph as rebotarm_moveit_config's demo.launch.py (robot_state_publisher,
ros2_control_node with mock_components/GenericSystem, joint_state_broadcaster +
rebotarm_controller + gripper_controller spawners, move_group) but with no RViz and no
GUI, so it can run on a headless CI/test box. Exiting move_group (e.g. via SIGINT to the
whole process group) cascades to ros2_control_node and then to a full LaunchService
shutdown, so nothing is left running.

Robot model (ADR-014): the vendor URDF/SRDF plus this repo's end-of-arm overlay
(crackvision_description: tool_tip, wrist D405 camera_link and padded camera/mount collision
proxies), placed from config/robot/end_effector.yaml. The config is validated before
MoveItConfigsBuilder runs, because the builder only *warns* when a description file is
missing and would otherwise start move_group without the camera geometry. Nominal
realsense optical frames are included because no camera driver runs in this mock stack.
"""

import os
import signal
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from crackvision_description.end_effector import load_config as load_end_effector_config
from launch import LaunchDescription
from launch.actions import EmitEvent, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown, matches_action
from launch.events.process import SignalProcess
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder


def _end_effector_config() -> Path:
    override = os.environ.get("CRACKVISION_END_EFFECTOR_CONFIG")
    if override:
        path = Path(override)
    else:
        root = os.environ.get("CRACKVISION_ROOT")
        if not root:
            raise RuntimeError(
                "CRACKVISION_ROOT is not set: run through scripts/ros/env_ros.sh (it exports the repo "
                "root) or set CRACKVISION_END_EFFECTOR_CONFIG to config/robot/end_effector.yaml"
            )
        path = Path(root) / "config" / "robot" / "end_effector.yaml"
    path = path.resolve()
    load_end_effector_config(path)  # raises EndEffectorError on a missing or invalid file
    return path


def generate_launch_description():
    end_effector_config = _end_effector_config()
    description_share = Path(get_package_share_directory("crackvision_description"))
    moveit_config = (
        MoveItConfigsBuilder("rebotarm", package_name="rebotarm_moveit_config")
        .robot_description(
            file_path=str(description_share / "urdf" / "b601_dm_end_effector.urdf.xacro"),
            mappings={
                "end_effector_config": str(end_effector_config),
                "nominal_camera_frames": "true",
            },
        )
        .robot_description_semantic(
            file_path=str(description_share / "srdf" / "b601_dm_end_effector.srdf.xacro"),
            mappings={"end_effector_config": str(end_effector_config)},
        )
        .robot_description_kinematics(file_path="config/kinematics.yaml")
        .joint_limits(file_path="config/joint_limits.yaml")
        .trajectory_execution(file_path="config/moveit_controllers.yaml")
        .planning_scene_monitor(
            publish_robot_description=True,
            publish_robot_description_semantic=True,
        )
        .planning_pipelines(pipelines=["ompl"])
        .to_moveit_configs()
    )

    static_tf_node = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="static_transform_publisher",
        output="log",
        arguments=["0", "0", "0", "0", "0", "0", "world", "base_link"],
    )

    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="both",
        parameters=[moveit_config.robot_description],
    )

    ros2_controllers_path = os.path.join(
        get_package_share_directory("rebotarm_moveit_config"),
        "config",
        "ros2_controllers.yaml",
    )
    ros2_control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[moveit_config.robot_description, ros2_controllers_path],
        output="screen",
    )

    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "joint_state_broadcaster",
            "--controller-manager",
            "/controller_manager",
        ],
    )

    rebotarm_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "rebotarm_controller",
            "--controller-manager",
            "/controller_manager",
        ],
    )

    gripper_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "gripper_controller",
            "--controller-manager",
            "/controller_manager",
        ],
    )

    move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[moveit_config.to_dict()],
    )

    return LaunchDescription(
        [
            static_tf_node,
            robot_state_publisher_node,
            ros2_control_node,
            joint_state_broadcaster_spawner,
            rebotarm_controller_spawner,
            gripper_controller_spawner,
            move_group_node,
            RegisterEventHandler(
                OnProcessExit(
                    target_action=move_group_node,
                    on_exit=[
                        EmitEvent(
                            event=SignalProcess(
                                signal_number=signal.SIGINT,
                                process_matcher=matches_action(ros2_control_node),
                            )
                        )
                    ],
                )
            ),
            RegisterEventHandler(
                OnProcessExit(
                    target_action=ros2_control_node,
                    on_exit=[
                        EmitEvent(event=Shutdown(reason="ros2_control_node exited"))
                    ],
                )
            ),
        ]
    )
