#!/usr/bin/env python3
import os
import xacro
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (IncludeLaunchDescription, RegisterEventHandler, TimerAction)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.event_handlers import OnProcessExit
from launch_ros.actions import Node


def generate_launch_description():
    pkg      = 'spiderbot'
    pkg_path = get_package_share_directory(pkg)
    world    = os.path.join(pkg_path, 'worlds', 'spiderbot.world')
    yaml     = os.path.join(pkg_path, 'config', 'controllers.yaml')
    xacro_f  = os.path.join(pkg_path, 'description', 'robot.urdf.xacro')

    robot_desc = xacro.process_file(
        xacro_f, mappings={'controllers_yaml': yaml}
    ).toxml()

    rsp = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_desc, 'use_sim_time': True}]
    )

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('gazebo_ros'),
                'launch', 'gazebo.launch.py'
            )
        ),
        launch_arguments={'world': world}.items()
    )

    spawn = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-topic', 'robot_description',
            '-entity', 'spiderbot',
            '-x', '0.0', '-y', '0.0', '-z', '0.14',
            '-R', '0.0', '-P', '0.0', '-Y', '0.0',
        ],
        output='screen'
    )

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

    # stabilize.py — runs at t=18s (after jtc is guaranteed active)
    stabilize = Node(
        package='spiderbot',
        executable='stabilize.py',
        output='screen',
    )

    return LaunchDescription([
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
        # Fixed timer: 18s total = plenty of time for all above to finish
        TimerAction(period=18.0, actions=[stabilize]),
    ])
