#!/usr/bin/env python3
import sys, math, time
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.duration import Duration
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint

JOINTS = [
    'fr1_joint','fr2_joint',
    'fl1_joint','fl2_joint',
    'rr1_joint','rr2_joint',
    'rl1_joint','rl2_joint',
]

# [fr1,fr2, fl1,fl2, rr1,rr2, rl1,rl2]
# joint1=coxa(yaw), joint2=tibia(pitch outward)
POSES = {
    'home'   : [0.0, 0.0,  0.0, 0.0,  0.0, 0.0,  0.0, 0.0],
    'stand'  : [0.0, 0.3,  0.0, 0.3,  0.0, 0.3,  0.0, 0.3],
    'sit'    : [0.0,-0.2,  0.0,-0.2,  0.0,-0.2,  0.0,-0.2],
    'stretch': [0.5, 0.0,  0.5, 0.0, -0.5, 0.0, -0.5, 0.0],
    'wave_fr': [0.0, 0.6,  0.0, 0.3,  0.0, 0.3,  0.0, 0.3],
    'wave_fl': [0.0, 0.3,  0.0, 0.6,  0.0, 0.3,  0.0, 0.3],
    'wave_rr': [0.0, 0.3,  0.0, 0.3,  0.0, 0.6,  0.0, 0.3],
    'wave_rl': [0.0, 0.3,  0.0, 0.3,  0.0, 0.3,  0.0, 0.6],
    'walk_a' : [0.3, 0.5,  0.0, 0.3,  0.0, 0.3,  0.3, 0.5],
    'walk_b' : [0.0, 0.3,  0.3, 0.5,  0.3, 0.5,  0.0, 0.3],
    'walk_c' : [-0.3,0.3,  0.0, 0.3,  0.0, 0.3, -0.3, 0.3],
    'walk_d' : [0.0, 0.3, -0.3, 0.3, -0.3, 0.3,  0.0, 0.3],
}

class SpiderController(Node):
    def __init__(self):
        super().__init__('spiderbot_controller')
        self._client = ActionClient(self, FollowJointTrajectory,
            '/joint_trajectory_controller/follow_joint_trajectory')
        self.get_logger().info('Waiting for controller...')
        self._client.wait_for_server()
        self.get_logger().info('Controller ready!')

    def move(self, positions, duration=1.5):
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = JOINTS
        pt = JointTrajectoryPoint()
        pt.positions       = [float(p) for p in positions]
        pt.velocities      = [0.0] * 8
        pt.time_from_start = Duration(seconds=duration).to_msg()
        goal.trajectory.points   = [pt]
        goal.goal_time_tolerance = Duration(seconds=1.0).to_msg()
        future = self._client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)
        handle = future.result()
        if not handle or not handle.accepted:
            self.get_logger().error('Goal REJECTED')
            return False
        rf = handle.get_result_async()
        rclpy.spin_until_future_complete(self, rf, timeout_sec=duration+2.0)
        self.get_logger().info('Done!')
        return True

    def move_pose(self, name, duration=1.5):
        if name not in POSES:
            self.get_logger().error('Unknown pose: {}  Try: {}'.format(name, list(POSES.keys())))
            return False
        self.get_logger().info('-> {}'.format(name))
        return self.move(POSES[name], duration)

def run_demo(node):
    print('\n=== SpiderBot Demo ===')
    for pose, dur, wait in [
        ('home',2.0,1.0),('stand',2.0,1.5),('sit',1.5,1.5),
        ('stand',1.5,1.0),('stretch',2.0,1.5),('stand',1.5,1.0),
        ('wave_fr',1.0,1.0),('stand',1.0,0.5),
        ('wave_fl',1.0,1.0),('stand',1.0,0.5),
        ('wave_rr',1.0,1.0),('stand',1.0,0.5),
        ('wave_rl',1.0,1.0),('stand',1.0,0.5),
        ('walk_a',0.6,0.3),('walk_b',0.6,0.3),
        ('walk_c',0.6,0.3),('walk_d',0.6,0.3),
        ('walk_a',0.6,0.3),('walk_b',0.6,0.3),
        ('stand',2.0,1.5),
    ]:
        print('  ->', pose)
        node.move_pose(pose, dur)
        time.sleep(wait)
    print('Demo complete!')

def main():
    rclpy.init()
    node = SpiderController()
    if len(sys.argv) > 1 and sys.argv[1] == 'demo':
        run_demo(node); rclpy.shutdown(); return
    print('\nPoses:', list(POSES.keys()))
    print('Commands: <pose>  |  j a1..a8(deg)  |  demo  |  list  |  q\n')
    while True:
        try:
            cmd = input('Command: ').strip()
            if not cmd: continue
            p = cmd.split()
            if p[0]=='q': break
            elif p[0]=='demo': run_demo(node)
            elif p[0]=='list':
                for k,v in POSES.items():
                    print('  {:10s} {}'.format(k,[round(x,2) for x in v]))
            elif p[0]=='j' and len(p)==9:
                node.move([math.radians(float(x)) for x in p[1:]])
            elif p[0] in POSES: node.move_pose(p[0])
            else: print('Unknown:', p[0])
        except KeyboardInterrupt: break
        except Exception as e: print('Error:', e)
    node.move_pose('stand', 2.0)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
