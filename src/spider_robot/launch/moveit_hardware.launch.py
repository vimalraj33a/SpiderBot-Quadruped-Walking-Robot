#!/usr/bin/env python3
"""
moveit_hardware.launch.py  — spider_robot  (workspace: spiderrobot_ws)
=======================================================================
Starts MoveIt2 + RViz alongside the hardware bridge.
Plan motions in RViz MotionPlanning panel → real servos execute.

Usage:
  # Terminal 1 — start simulation first
  ros2 launch spider_robot hardware.launch.py port:=/dev/ttyUSB0

  # Terminal 2 — start MoveIt + RViz
  ros2 launch spider_robot moveit_hardware.launch.py port:=/dev/ttyUSB0
"""

import os
import yaml
import xacro
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def load_yaml(pkg, rel_path):
    full = os.path.join(get_package_share_directory(pkg), rel_path)
    with open(full, 'r') as f:
        return yaml.safe_load(f)


def generate_launch_description():

    port_arg    = DeclareLaunchArgument('port',    default_value='/dev/ttyUSB0')
    dry_run_arg = DeclareLaunchArgument('dry_run', default_value='false')
    port    = LaunchConfiguration('port')
    dry_run = LaunchConfiguration('dry_run')

    spiderbot_pkg = get_package_share_directory('spiderbot')
    xacro_file = os.path.join(spiderbot_pkg, 'description', 'robot.urdf.xacro')
    ctrl_yaml  = os.path.join(spiderbot_pkg, 'config',      'controllers.yaml')

    robot_desc_xml = xacro.process_file(
        xacro_file, mappings={'controllers_yaml': ctrl_yaml}
    ).toxml()

    srdf         = load_yaml('spiderbot', 'config/moveit/spiderbot.srdf')
    kinematics   = load_yaml('spiderbot', 'config/moveit/kinematics.yaml')
    joint_limits = load_yaml('spiderbot', 'config/moveit/joint_limits.yaml')
    planning     = load_yaml('spiderbot', 'config/moveit/planning_pipeline.yaml')
    mv_ctrl      = load_yaml('spiderbot', 'config/moveit/moveit_controllers.yaml')

    move_group = Node(
        package='moveit_ros_move_group',
        executable='move_group',
        output='screen',
        parameters=[{
            'robot_description':             robot_desc_xml,
            'robot_description_semantic':    yaml.dump(srdf),
            'robot_description_kinematics':  kinematics,
            'robot_description_planning':    joint_limits,
            'use_sim_time': True,
            **planning,
            **mv_ctrl,
        }]
    )

    rviz_cfg = os.path.join(spiderbot_pkg, 'rviz', 'moveit.rviz')
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', rviz_cfg],
        parameters=[{
            'robot_description':          robot_desc_xml,
            'robot_description_semantic': yaml.dump(srdf),
            'use_sim_time': True,
        }],
        output='screen'
    )

    hw_bridge = Node(
        package='spider_robot',
        executable='hardware_bridge.py',
        output='screen',
        parameters=[{
            'port':            port,
            'baud':            115200,
            'dry_run':         dry_run,
            'publish_rate_hz': 20.0,
        }]
    )

    return LaunchDescription([
        port_arg,
        dry_run_arg,
        move_group,
        rviz,
        TimerAction(period=5.0, actions=[hw_bridge]),
    ])
