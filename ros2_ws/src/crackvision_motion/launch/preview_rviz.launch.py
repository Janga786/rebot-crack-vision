"""preview_rviz.launch.py -- RViz preview of a trajectory file, motion-free (§18.4, MOT-08.7).

Composes mock_planning.launch.py (MOT-02's headless mock stack, started purely so RViz has a
RobotModel/PlanningScene to render against, never as a motion path) with rviz2 -d
<share>/rviz/preview.rviz and the preview_trajectory node from MOT-08.6. The mock stack's own
teardown cascade (move_group exit -> ros2_control_node SIGINT -> Shutdown) is unchanged by
including it here.

Manual operator use (requires a display):
    ros2 launch crackvision_motion preview_rviz.launch.py trajectory:=PATH

Headless smoke test (no display): scripts/ros/test_preview_rviz.sh runs the mock stack and the
preview_trajectory node directly with --once, bypassing rviz2 (rviz:=false), and verifies the
published topics with an rclpy subscriber instead of visually.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node


def _preview_trajectory_node(context, *args, **kwargs):
    node_args = ["--trajectory", LaunchConfiguration("trajectory").perform(context)]
    paths3d = LaunchConfiguration("paths3d").perform(context)
    if paths3d:
        node_args += ["--paths3d", paths3d]
    speed_scale = LaunchConfiguration("speed_scale").perform(context)
    if speed_scale:
        node_args += ["--speed-scale", speed_scale]
    return [
        Node(
            package="crackvision_motion",
            executable="preview_trajectory",
            name="preview_trajectory",
            output="screen",
            arguments=node_args,
        )
    ]


def generate_launch_description():
    trajectory_arg = DeclareLaunchArgument(
        "trajectory", description="crackvision.joint_trajectory/1 file to preview"
    )
    paths3d_arg = DeclareLaunchArgument(
        "paths3d", default_value="", description="optional crackvision.paths3d/1 crack overlay"
    )
    speed_scale_arg = DeclareLaunchArgument(
        "speed_scale", default_value="",
        description="optional speed scale override; default is preview_trajectory's own "
                     "(execution.yaml speed_scale.real_default)",
    )
    rviz_arg = DeclareLaunchArgument(
        "rviz", default_value="true",
        description="start rviz2 with rviz/preview.rviz; set false for headless use",
    )

    motion_share = get_package_share_directory("crackvision_motion")

    mock_planning = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(motion_share, "launch", "mock_planning.launch.py")
        )
    )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=["-d", PathJoinSubstitution([motion_share, "rviz", "preview.rviz"])],
        condition=IfCondition(LaunchConfiguration("rviz")),
    )

    return LaunchDescription(
        [
            trajectory_arg,
            paths3d_arg,
            speed_scale_arg,
            rviz_arg,
            mock_planning,
            rviz_node,
            OpaqueFunction(function=_preview_trajectory_node),
        ]
    )
