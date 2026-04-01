#!/usr/bin/env python3
import sys, tty, termios, threading, time
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.duration import Duration
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint

JOINTS = ['fr1_joint','fr2_joint','fl1_joint','fl2_joint',
          'rr1_joint','rr2_joint','rl1_joint','rl2_joint']

POSES = {
    'stand' : [0.0,0.4, 0.0,0.4, 0.0,0.4, 0.0,0.4],
    'sit'   : [0.0,0.1, 0.0,0.1, 0.0,0.1, 0.0,0.1],
    'stretch': [0.5,-0.1,0.5,-0.1,-0.5,-0.1,-0.5,-0.1],
    'walk_a': [0.3,0.1, 0.0,0.4, 0.0,0.4, 0.3,0.1],
    'walk_b': [0.0,0.4, 0.3,0.1, 0.3,0.1, 0.0,0.4],
    'walk_c': [-0.3,0.4,0.0,0.4, 0.0,0.4,-0.3,0.4],
    'walk_d': [0.0,0.4,-0.3,0.4,-0.3,0.4, 0.0,0.4],
}

class TeleopNode(Node):
    def __init__(self):
        super().__init__('spiderbot_teleop')
        self._c = ActionClient(self, FollowJointTrajectory,
            '/joint_trajectory_controller/follow_joint_trajectory')
        self.get_logger().info('Waiting for controller...')
        self._c.wait_for_server()
        self.joints = list(POSES['stand'])
        self.selected = 0
        self.get_logger().info('Teleop ready!')

    def send(self, pos, dur=0.8):
        g = FollowJointTrajectory.Goal()
        g.trajectory.joint_names = JOINTS
        pt = JointTrajectoryPoint()
        pt.positions = [float(p) for p in pos]
        pt.velocities = [0.0]*8
        pt.time_from_start = Duration(seconds=dur).to_msg()
        g.trajectory.points = [pt]
        f = self._c.send_goal_async(g)
        rclpy.spin_until_future_complete(self, f, timeout_sec=3.0)
        h = f.result()
        if h and h.accepted:
            rf = h.get_result_async()
            rclpy.spin_until_future_complete(self, rf, timeout_sec=dur+1.0)

def get_key():
    fd = sys.stdin.fileno()
    old = termios.tcgetattr(fd)
    try:
        tty.setraw(fd)
        ch = sys.stdin.read(1)
        if ch == '\x1b':
            ch += sys.stdin.read(2)
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old)
    return ch

def main():
    rclpy.init()
    node = TeleopNode()
    t = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    t.start()
    node.send(POSES['stand'], 2.0)
    print('\n=== SpiderBot Keyboard Teleop ===')
    print('W: walk fwd   S: walk back   SPACE: stand')
    print('Z: sit   X: stretch   1/2/3/4: select FR/FL/RR/RL')
    print('Arrow UP/DN: tibia   Arrow L/R: coxa   Q: quit\n')
    while True:
        try:
            k = get_key()
            if k in ('q','Q','\x03'): break
            elif k==' ':    node.send(POSES['stand'])
            elif k=='z':    node.send(POSES['sit'])
            elif k=='x':    node.send(POSES['stretch'])
            elif k=='w':
                for p in ['walk_a','walk_c','walk_b','walk_d']:
                    node.send(POSES[p],0.5); time.sleep(0.1)
            elif k=='s':
                for p in ['walk_c','walk_a','walk_d','walk_b']:
                    node.send(POSES[p],0.5); time.sleep(0.1)
            elif k=='1': node.selected=0; print('Leg: FR')
            elif k=='2': node.selected=1; print('Leg: FL')
            elif k=='3': node.selected=2; print('Leg: RR')
            elif k=='4': node.selected=3; print('Leg: RL')
            elif k=='\x1b[A':  # up — raise tibia
                i = node.selected*2+1
                node.joints[i] = max(-1.57, node.joints[i]-0.1)
                node.send(node.joints)
            elif k=='\x1b[B':  # down — lower tibia
                i = node.selected*2+1
                node.joints[i] = min(1.57, node.joints[i]+0.1)
                node.send(node.joints)
            elif k=='\x1b[C':  # right — coxa +
                i = node.selected*2
                node.joints[i] = min(1.57, node.joints[i]+0.1)
                node.send(node.joints)
            elif k=='\x1b[D':  # left — coxa -
                i = node.selected*2
                node.joints[i] = max(-1.57, node.joints[i]-0.1)
                node.send(node.joints)
        except Exception as e:
            print('Error:', e); break
    node.send(POSES['stand'], 2.0)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
