# Copyright 2026 frdedynamics
# SPDX-License-Identifier: Apache-2.0
"""Fixed D435i identities; RGB baseline with opt-in depth."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    driver_launch = PathJoinSubstitution([
        FindPackageShare('realsense2_camera'), 'launch', 'rs_launch.py',
    ])
    cameras = []
    for name, serial in (
        ('wrist_camera', '342222073510'),
        ('external_camera', '244222076317'),
    ):
        # Separate scopes prevent the upstream launch arguments from leaking
        # between devices. The leading underscore keeps serial_no a string
        # through ROS launch YAML conversion; the RealSense driver removes it.
        cameras.append(GroupAction(scoped=True, actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(driver_launch),
                launch_arguments={
                    'camera_namespace': 'camera',
                    'camera_name': name,
                    'serial_no': '_' + serial,
                    'enable_color': 'true',
                    'rgb_camera.color_profile': '1280x720x30',
                    'rgb_camera.color_format': 'RGB8',
                    'enable_depth': LaunchConfiguration('enable_depth'),
                    'enable_gyro': 'false',
                    'enable_accel': 'false',
                }.items(),
            ),
        ]))
    return LaunchDescription([
        DeclareLaunchArgument(
            'enable_depth', default_value='false', choices=['true', 'false'],
            description='Enable depth on both cameras; RGB-only by default.',
        ),
        *cameras,
    ])
