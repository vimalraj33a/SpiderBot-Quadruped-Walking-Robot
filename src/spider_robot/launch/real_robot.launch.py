#!/usr/bin/env python3
"""
real_robot.launch.py  — spider_robot  (workspace: spiderrobot_ws)
==================================================================
Run WITHOUT Gazebo. Publishes robot state and bridges joint commands
directly to the real ESP32 hardware over USB.

Use this when moveit_control.py / walk.py / keyboard_teleop.py should
drive the real robot WITHOUT simulation running.

Usage:
  ros2 launch spider_robot real_robot.launch.py
  ros2 launch spider_robot real_robot.launch.py port:=/dev/ttyUSB1
  ros2 launch spider_robot real_robot.launch.py dry_run:=true
"""

import os
import xacro
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():

    port_arg    = DeclareLaunchArgument('port',    default_value='/dev/ttyUSB0')
    baud_arg    = DeclareLaunchArgument('baud',    default_value='115200')
    dry_run_arg = DeclareLaunchArgument('dry_run', default_value='false')

    port    = LaunchConfiguration('port')
    baud    = LaunchConfiguration('baud')
    dry_run = LaunchConfiguration('dry_run')

    spiderbot_pkg = get_package_share_directory('spiderbot')
    xacro_file = os.path.join(spiderbot_pkg, 'description', 'robot.urdf.xacro')
    ctrl_yaml  = os.path.join(spiderbot_pkg, 'config',      'controllers.yaml')

    robot_desc = xacro.process_file(
        xacro_file, mappings={'controllers_yaml': ctrl_yaml}
    ).toxml()

    # RSP without sim time — real robot clock
    rsp = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': robot_desc,
            'use_sim_time': False,
        }]
    )

    # Hardware bridge — listens to /joint_states, sends to ESP32
    hw_bridge = Node(
        package='spider_robot',
        executable='hardware_bridge.py',
        output='screen',
        parameters=[{
            'port':            port,
            'baud':            baud,
            'dry_run':         dry_run,
            'publish_rate_hz': 25.0,
        }]
    )

    return LaunchDescription([
        port_arg,
        baud_arg,
        dry_run_arg,
        rsp,
        hw_bridge,
    ])
