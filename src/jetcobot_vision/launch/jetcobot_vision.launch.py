"""
jetcobot_vision.launch.py
=========================
세 노드를 한 번에 기동합니다:
  1. detect_bridge_node  — cv_detect_server /latest 폴링 → /detect_bridge/obb_boxes
  2. coord_transform_node — Static TF + 역투영 → /coord_transform/pick_point_base
  3. vision_pick_node     — Action Server /vision_pick

사용법:
  ros2 launch jetcobot_vision jetcobot_vision.launch.py
  ros2 launch jetcobot_vision jetcobot_vision.launch.py params_file:=/path/to/custom.yaml
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    pkg_dir     = get_package_share_directory("jetcobot_vision")
    default_cfg = os.path.join(pkg_dir, "config", "vision_params.yaml")

    params_arg = DeclareLaunchArgument(
        "params_file",
        default_value=default_cfg,
        description="파라미터 YAML 파일 절대 경로",
    )
    params = LaunchConfiguration("params_file")

    return LaunchDescription([
        params_arg,
        Node(
            package="jetcobot_vision",
            executable="detect_bridge_node",
            name="detect_bridge",
            parameters=[params],
            output="screen",
            emulate_tty=True,
        ),
        Node(
            package="jetcobot_vision",
            executable="coord_transform_node",
            name="coord_transform_node",
            parameters=[params],
            output="screen",
            emulate_tty=True,
        ),
        Node(
            package="jetcobot_vision",
            executable="vision_pick_node",
            name="vision_pick_node",
            parameters=[params],
            output="screen",
            emulate_tty=True,
        ),
    ])
