#!/usr/bin/env python3
"""
moveit.launch.py — MoveIt move_group + clean RViz2
Run AFTER launch_sim.launch.py controllers are active.
"""
import os, yaml
import xacro
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def load_yaml(pkg_path, rel_path):
    with open(os.path.join(pkg_path, rel_path)) as f:
        return yaml.safe_load(f)


def generate_launch_description():
    pkg_path = get_package_share_directory('spiderbot')

    robot_desc_xml = xacro.process_file(
        os.path.join(pkg_path, 'description', 'robot.urdf.xacro')
    ).toxml()
    robot_description = {'robot_description': robot_desc_xml}

    with open(os.path.join(pkg_path, 'config', 'moveit', 'spiderbot.srdf')) as f:
        srdf = f.read()
    robot_description_semantic = {'robot_description_semantic': srdf}

    kinematics   = load_yaml(pkg_path, 'config/moveit/kinematics.yaml')
    joint_limits = load_yaml(pkg_path, 'config/moveit/joint_limits.yaml')
    controllers  = load_yaml(pkg_path, 'config/moveit/moveit_controllers.yaml')

    planning_pipeline = {
        'planning_plugin': 'ompl_interface/OMPLPlanner',
        'request_adapters': (
            'default_planner_request_adapters/AddTimeOptimalParameterization '
            'default_planner_request_adapters/ResolveConstraintFrames '
            'default_planner_request_adapters/FixWorkspaceBounds '
            'default_planner_request_adapters/FixStartStateBounds '
            'default_planner_request_adapters/FixStartStateCollision'
        ),
        'start_state_max_bounds_error': 0.1,
    }

    move_group = Node(
        package='moveit_ros_move_group',
        executable='move_group',
        output='screen',
        parameters=[
            robot_description,
            robot_description_semantic,
            {'robot_description_kinematics': kinematics},
            {'robot_description_planning': joint_limits},
            planning_pipeline,
            controllers,
            {
                'moveit_manage_controllers': True,
                'trajectory_execution.allowed_execution_duration_scaling': 1.2,
                'trajectory_execution.allowed_goal_duration_margin': 0.5,
                'trajectory_execution.allowed_start_tolerance': 0.05,
                'publish_planning_scene': True,
                'publish_geometry_updates': True,
                'publish_state_updates': True,
                'use_sim_time': True,
            },
        ],
    )

    # Plain RViz2 with robot model + TF (no broken MoveIt panel)
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', os.path.join(pkg_path, 'rviz', 'moveit.rviz')],
        parameters=[
            robot_description,
            {'use_sim_time': True},
        ],
        output='screen',
    )

    static_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=['0', '0', '0', '0', '0', '0', 'world', 'base_link'],
        parameters=[{'use_sim_time': True}],
    )

    return LaunchDescription([static_tf, move_group, rviz])
