#!/usr/bin/env python3
import os
import xacro
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    pkg_path = get_package_share_directory('spiderbot')
    xacro_f  = os.path.join(pkg_path, 'description', 'robot.urdf.xacro')
    rviz_cfg = os.path.join(pkg_path, 'rviz', 'default.rviz')

    robot_desc = xacro.process_file(xacro_f).toxml()

    rsp = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': robot_desc}]
    )
    jsp = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui'
    )
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', rviz_cfg]
    )

    return LaunchDescription([rsp, jsp, rviz])
