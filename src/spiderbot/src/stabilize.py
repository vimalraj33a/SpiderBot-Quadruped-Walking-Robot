#!/usr/bin/env python3
"""
stabilize.py — Runs once at startup to lock robot in place.
Called automatically from launch_sim.launch.py after controllers load.
"""
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.duration import Duration
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint
import sys

JOINTS = [
    'fr1_joint','fr2_joint',
    'fl1_joint','fl2_joint',
    'rr1_joint','rr2_joint',
    'rl1_joint','rl2_joint',
]

# All joints at zero = legs straight out, feet on ground
ZERO = [0.0, 0.0,  0.0, 0.0,  0.0, 0.0,  0.0, 0.0]

class Stabilizer(Node):
    def __init__(self):
        super().__init__('spiderbot_stabilizer')
        self._client = ActionClient(
            self, FollowJointTrajectory,
            '/joint_trajectory_controller/follow_joint_trajectory'
        )

    def run(self):
        self.get_logger().info('Waiting for controller...')
        self._client.wait_for_server(timeout_sec=15.0)
        self.get_logger().info('Sending zero position to stop robot...')

        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = JOINTS

        pt = JointTrajectoryPoint()
        pt.positions       = ZERO
        pt.velocities      = [0.0] * 8
        pt.accelerations   = [0.0] * 8
        pt.time_from_start = Duration(seconds=2.0).to_msg()

        goal.trajectory.points   = [pt]
        goal.goal_time_tolerance = Duration(seconds=2.0).to_msg()

        future = self._client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, future, timeout_sec=10.0)

        handle = future.result()
        if handle and handle.accepted:
            result_f = handle.get_result_async()
            rclpy.spin_until_future_complete(self, result_f, timeout_sec=5.0)
            self.get_logger().info('Robot stabilized at center!')
        else:
            self.get_logger().error('Goal rejected — robot may still move')

def main():
    rclpy.init()
    node = Stabilizer()
    node.run()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
