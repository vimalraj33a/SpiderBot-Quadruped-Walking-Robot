#!/usr/bin/env python3
"""
hardware.launch.py  — spider_robot  (workspace: spiderrobot_ws)
================================================================
Starts Gazebo simulation AND mirrors every joint movement to the
real spider robot hardware over USB serial → ESP32 → PCA9685 → servos.

Usage:
  ros2 launch spider_robot hardware.launch.py
  ros2 launch spider_robot hardware.launch.py port:=/dev/ttyUSB1
  ros2 launch spider_robot hardware.launch.py dry_run:=true
"""

import os
import xacro
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    IncludeLaunchDescription,
    RegisterEventHandler,
    TimerAction,
    DeclareLaunchArgument,
)
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.event_handlers import OnProcessExit
from launch_ros.actions import Node


def generate_launch_description():

    # ── Launch arguments ──────────────────────────────────────────────
    port_arg    = DeclareLaunchArgument('port',    default_value='/dev/ttyUSB0',
                                        description='ESP32 USB serial port')
    baud_arg    = DeclareLaunchArgument('baud',    default_value='115200',
                                        description='Serial baud rate')
    dry_run_arg = DeclareLaunchArgument('dry_run', default_value='false',
                                        description='true = log only, no serial output')

    port    = LaunchConfiguration('port')
    baud    = LaunchConfiguration('baud')
    dry_run = LaunchConfiguration('dry_run')

    # ── Package paths (resolved from spiderrobot_ws install) ──────────
    spiderbot_pkg = get_package_share_directory('spiderbot')

    xacro_file = os.path.join(spiderbot_pkg, 'description', 'robot.urdf.xacro')
    ctrl_yaml  = os.path.join(spiderbot_pkg, 'config',      'controllers.yaml')
    world_file = os.path.join(spiderbot_pkg, 'worlds',      'spiderbot.world')

    robot_desc = xacro.process_file(
        xacro_file, mappings={'controllers_yaml': ctrl_yaml}
    ).toxml()

    # ── Robot State Publisher ──────────────────────────────────────────
    rsp = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': robot_desc,
            'use_sim_time': True,
        }]
    )

    # ── Gazebo ────────────────────────────────────────────────────────
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('gazebo_ros'),
                'launch', 'gazebo.launch.py'
            )
        ),
        launch_arguments={'world': world_file}.items()
    )

    # ── Spawn robot (wait 8 s for Gazebo to fully load) ───────────────
    spawn = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-topic', 'robot_description',
            '-entity', 'spiderbot',
            '-x', '0.0', '-y', '0.0', '-z', '0.14',
        ],
        output='screen'
    )

    # ── Controller spawners ───────────────────────────────────────────
    jsb = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster',
                   '--controller-manager', '/controller_manager',
                   '--controller-manager-timeout', '30'],
        output='screen'
    )

    jtc = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_trajectory_controller',
                   '--controller-manager', '/controller_manager',
                   '--controller-manager-timeout', '30'],
        output='screen'
    )

    # ── Stabilizer — send zero pose after controllers load ────────────
    stabilize = Node(
        package='spiderbot',
        executable='stabilize.py',
        output='screen',
    )

    # ── Hardware bridge — forward /joint_states → ESP32 via USB ───────
    hw_bridge = Node(
        package='spider_robot',
        executable='hardware_bridge.py',
        output='screen',
        parameters=[{
            'port':             port,
            'baud':             baud,
            'dry_run':          dry_run,
            'publish_rate_hz':  20.0,
        }]
    )

    return LaunchDescription([
        port_arg,
        baud_arg,
        dry_run_arg,

        rsp,
        gazebo,

        TimerAction(period=8.0,  actions=[spawn]),

        RegisterEventHandler(
            OnProcessExit(target_action=spawn, on_exit=[
                TimerAction(period=3.0, actions=[jsb])
            ])
        ),

        RegisterEventHandler(
            OnProcessExit(target_action=jsb, on_exit=[jtc])
        ),

        TimerAction(period=18.0, actions=[stabilize]),
        TimerAction(period=20.0, actions=[hw_bridge]),
    ])
