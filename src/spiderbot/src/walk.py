#!/usr/bin/env python3
import time, rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.duration import Duration
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint

JOINTS = ['fr1_joint','fr2_joint','fl1_joint','fl2_joint',
          'rr1_joint','rr2_joint','rl1_joint','rl2_joint']

STAND  = [0.0,0.4, 0.0,0.4, 0.0,0.4, 0.0,0.4]
WALK_A = [0.3,0.1, 0.0,0.4, 0.0,0.4, 0.3,0.1]
WALK_B = [0.0,0.4, 0.3,0.1, 0.3,0.1, 0.0,0.4]
WALK_C = [-0.3,0.4,0.0,0.4, 0.0,0.4,-0.3,0.4]
WALK_D = [0.0,0.4,-0.3,0.4,-0.3,0.4, 0.0,0.4]

class Walker(Node):
    def __init__(self):
        super().__init__('spiderbot_walker')
        self._c = ActionClient(self, FollowJointTrajectory,
            '/joint_trajectory_controller/follow_joint_trajectory')
        self.get_logger().info('Waiting...')
        self._c.wait_for_server()

    def send(self, pos, dur=0.6):
        g = FollowJointTrajectory.Goal()
        g.trajectory.joint_names = JOINTS
        pt = JointTrajectoryPoint()
        pt.positions = [float(p) for p in pos]
        pt.velocities = [0.0]*8
        pt.time_from_start = Duration(seconds=dur).to_msg()
        g.trajectory.points = [pt]
        f = self._c.send_goal_async(g)
        rclpy.spin_until_future_complete(self, f, timeout_sec=5.0)
        h = f.result()
        if h and h.accepted:
            rf = h.get_result_async()
            rclpy.spin_until_future_complete(self, rf, timeout_sec=dur+2.0)

def main():
    rclpy.init()
    node = Walker()
    print('Walking... Ctrl+C to stop')
    node.send(STAND, 2.0); time.sleep(2.0)
    try:
        while True:
            for pose in [WALK_A, WALK_C, WALK_B, WALK_D]:
                node.send(pose, 0.5); time.sleep(0.3)
    except KeyboardInterrupt:
        node.send(STAND, 1.5)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
