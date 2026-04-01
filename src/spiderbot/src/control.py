#!/usr/bin/env python3
import sys, math, rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.duration import Duration
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint

JOINTS = ['fr1_joint','fr2_joint','fl1_joint','fl2_joint',
          'rr1_joint','rr2_joint','rl1_joint','rl2_joint']

class ControlNode(Node):
    def __init__(self):
        super().__init__('spiderbot_control')
        self._c = ActionClient(self, FollowJointTrajectory,
            '/joint_trajectory_controller/follow_joint_trajectory')
        self._c.wait_for_server()

    def send(self, positions, duration=2.0):
        g = FollowJointTrajectory.Goal()
        g.trajectory.joint_names = JOINTS
        pt = JointTrajectoryPoint()
        pt.positions = [float(p) for p in positions]
        pt.velocities = [0.0]*8
        pt.time_from_start = Duration(seconds=duration).to_msg()
        g.trajectory.points = [pt]
        f = self._c.send_goal_async(g)
        rclpy.spin_until_future_complete(self, f)
        h = f.result()
        if h and h.accepted:
            rf = h.get_result_async()
            rclpy.spin_until_future_complete(self, rf, timeout_sec=duration+2.0)
        self.get_logger().info('Done')

def main():
    rclpy.init()
    node = ControlNode()
    args = {a.split(':=')[0]:float(a.split(':=')[1]) for a in sys.argv[1:] if ':=' in a}
    pos = [math.radians(args.get('ax{}'.format(i+1),0.0)) for i in range(8)]
    node.send(pos)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
