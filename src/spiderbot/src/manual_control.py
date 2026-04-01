#!/usr/bin/env python3
"""
manual_control.py
=================
Manual control of SpiderBot — 3 methods:
  1. Slider GUI  (tkinter sliders for all 8 joints)
  2. Keyboard    (WASD + arrow keys)
  3. CLI         (type joint angles directly)

Usage:
  ros2 run spiderbot manual_control.py          # GUI sliders
  ros2 run spiderbot manual_control.py keyboard # keyboard
  ros2 run spiderbot manual_control.py cli      # command line
"""
import sys, math, time, threading
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.duration import Duration
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint

# ─── Joint names ───────────────────────────────────────────
JOINTS = [
    'fr1_joint', 'fr2_joint',   # Front-Right  coxa, tibia
    'fl1_joint', 'fl2_joint',   # Front-Left   coxa, tibia
    'rr1_joint', 'rr2_joint',   # Rear-Right   coxa, tibia
    'rl1_joint', 'rl2_joint',   # Rear-Left    coxa, tibia
]

JOINT_LABELS = [
    'FR Coxa (yaw)', 'FR Tibia (pitch)',
    'FL Coxa (yaw)', 'FL Tibia (pitch)',
    'RR Coxa (yaw)', 'RR Tibia (pitch)',
    'RL Coxa (yaw)', 'RL Tibia (pitch)',
]

# ─── Preset poses ──────────────────────────────────────────
POSES = {
    'stand'  : [0.0, 0.3,  0.0, 0.3,  0.0, 0.3,  0.0, 0.3],
    'sit'    : [0.0,-0.2,  0.0,-0.2,  0.0,-0.2,  0.0,-0.2],
    'home'   : [0.0, 0.0,  0.0, 0.0,  0.0, 0.0,  0.0, 0.0],
    'stretch': [0.5,-0.1,  0.5,-0.1, -0.5,-0.1, -0.5,-0.1],
    'wave_fr': [0.7, 0.0,  0.0, 0.4,  0.0, 0.4,  0.0, 0.4],
    'wave_fl': [0.0, 0.4,  0.7, 0.0,  0.0, 0.4,  0.0, 0.4],
    'walk_a' : [0.3, 0.1,  0.0, 0.4,  0.0, 0.4,  0.3, 0.1],
    'walk_b' : [0.0, 0.4,  0.3, 0.1,  0.3, 0.1,  0.0, 0.4],
}


# ─── ROS2 Controller Node ──────────────────────────────────
class RobotNode(Node):
    def __init__(self):
        super().__init__('spiderbot_manual')
        self._client = ActionClient(
            self, FollowJointTrajectory,
            '/joint_trajectory_controller/follow_joint_trajectory'
        )
        self.get_logger().info('Connecting to controller...')
        self._client.wait_for_server()
        self.get_logger().info('Connected!')
        self.current = list(POSES['stand'])

    def send(self, positions, duration=0.8):
        """Send joint positions (radians) to robot."""
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = JOINTS
        pt = JointTrajectoryPoint()
        pt.positions       = [float(p) for p in positions]
        pt.velocities      = [0.0] * 8
        pt.time_from_start = Duration(seconds=duration).to_msg()
        goal.trajectory.points   = [pt]
        goal.goal_time_tolerance = Duration(seconds=0.5).to_msg()
        future = self._client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, future, timeout_sec=3.0)
        handle = future.result()
        if handle and handle.accepted:
            self.current = list(positions)
            rf = handle.get_result_async()
            rclpy.spin_until_future_complete(self, rf, timeout_sec=duration+1.0)
            return True
        return False

    def send_degrees(self, deg_list, duration=0.8):
        """Send joint positions in degrees."""
        return self.send([math.radians(d) for d in deg_list], duration)

    def pose(self, name, duration=1.5):
        if name in POSES:
            print('Pose: {}'.format(name))
            return self.send(POSES[name], duration)
        print('Unknown pose:', name)
        return False


# ══════════════════════════════════════════════════════════
# METHOD 1: GUI SLIDER CONTROL
# ══════════════════════════════════════════════════════════
def run_gui(node):
    try:
        import tkinter as tk
        from tkinter import ttk
    except ImportError:
        print('tkinter not found. Install: sudo apt install python3-tk')
        return

    root = tk.Tk()
    root.title('SpiderBot Manual Control')
    root.configure(bg='#1e1e2e')
    root.geometry('520x700')
    root.resizable(False, True)

    # Colors
    BG   = '#1e1e2e'
    FG   = '#cdd6f4'
    ACC  = '#89b4fa'
    RED  = '#f38ba8'
    GRN  = '#a6e3a1'
    YEL  = '#f9e2af'
    CARD = '#313244'

    sliders = []
    vars_   = []

    def send_current(*_):
        vals = [v.get() for v in vars_]
        node.send([math.radians(v) for v in vals], duration=0.5)
        # Update value labels
        for i, v in enumerate(vals):
            val_labels[i].config(text='{:+.1f}°'.format(v))

    def set_pose(name):
        angles_deg = [math.degrees(v) for v in POSES[name]]
        for i, v in enumerate(vars_):
            v.set(angles_deg[i])
        send_current()

    # ── Title ──
    tk.Label(root, text='🕷  SpiderBot Control',
             font=('Arial', 16, 'bold'),
             bg=BG, fg=ACC).pack(pady=(12, 4))
    tk.Label(root, text='Drag sliders to move joints | Buttons for presets',
             font=('Arial', 9), bg=BG, fg=FG).pack(pady=(0, 8))

    # ── Sliders ──
    slider_frame = tk.Frame(root, bg=BG)
    slider_frame.pack(fill='x', padx=16)

    leg_colors = [ACC, ACC, GRN, GRN, YEL, YEL, RED, RED]
    val_labels = []

    for i, (label, color) in enumerate(zip(JOINT_LABELS, leg_colors)):
        row = tk.Frame(slider_frame, bg=CARD, pady=4)
        row.pack(fill='x', pady=3, padx=2)

        tk.Label(row, text=label, width=20, anchor='w',
                 font=('Arial', 9, 'bold'),
                 bg=CARD, fg=color).pack(side='left', padx=8)

        var = tk.DoubleVar(value=math.degrees(POSES['stand'][i]))
        vars_.append(var)

        sl = tk.Scale(
            row, variable=var,
            from_=-90, to=90,
            orient='horizontal', length=220,
            resolution=0.5, showvalue=False,
            bg=CARD, fg=FG,
            highlightthickness=0,
            troughcolor='#45475a',
            activebackground=color,
            command=send_current
        )
        sl.pack(side='left', padx=4)
        sliders.append(sl)

        lbl = tk.Label(row, text='+0.0°', width=7,
                       font=('Courier', 9),
                       bg=CARD, fg=FG)
        lbl.pack(side='left', padx=4)
        val_labels.append(lbl)

    # ── Preset Buttons ──
    tk.Label(root, text='── Presets ──',
             font=('Arial', 10, 'bold'),
             bg=BG, fg=FG).pack(pady=(12, 4))

    btn_frame = tk.Frame(root, bg=BG)
    btn_frame.pack()

    btn_cfg = [
        ('Stand',   'stand',   GRN),
        ('Sit',     'sit',     YEL),
        ('Home',    'home',    ACC),
        ('Stretch', 'stretch', RED),
        ('Wave FR', 'wave_fr', ACC),
        ('Wave FL', 'wave_fl', GRN),
        ('Walk A',  'walk_a',  YEL),
        ('Walk B',  'walk_b',  RED),
    ]

    for i, (text, pose_name, color) in enumerate(btn_cfg):
        btn = tk.Button(
            btn_frame, text=text,
            width=10, height=1,
            font=('Arial', 9, 'bold'),
            bg=CARD, fg=color,
            activebackground=color,
            activeforeground=BG,
            relief='flat',
            cursor='hand2',
            command=lambda p=pose_name: set_pose(p)
        )
        btn.grid(row=i//4, column=i%4, padx=5, pady=4)

    # ── Joint Reset ──
    def reset_all():
        for v in vars_:
            v.set(0.0)
        send_current()

    tk.Button(
        root, text='⬛  Reset All to Zero',
        width=24, height=1,
        font=('Arial', 9, 'bold'),
        bg='#45475a', fg=FG,
        activebackground=FG,
        activeforeground=BG,
        relief='flat', cursor='hand2',
        command=reset_all
    ).pack(pady=10)

    # ── Status ──
    status_var = tk.StringVar(value='Ready — drag sliders to move robot')
    tk.Label(root, textvariable=status_var,
             font=('Arial', 8), bg=BG, fg='#6c7086').pack(pady=4)

    root.mainloop()


# ══════════════════════════════════════════════════════════
# METHOD 2: KEYBOARD CONTROL
# ══════════════════════════════════════════════════════════
def run_keyboard(node):
    import tty, termios

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

    # Start with standing pose
    joints = list(POSES['stand'])
    selected_leg  = 0   # 0=FR 1=FL 2=RR 3=RL
    step = 0.08         # radians per keypress
    LEG_NAMES = ['FR', 'FL', 'RR', 'RL']

    def print_status():
        print('\033[2J\033[H', end='')  # clear screen
        print('╔══════════════════════════════════════════╗')
        print('║       SpiderBot Keyboard Control         ║')
        print('╠══════════════════════════════════════════╣')
        print('║  W/S    : Walk forward / backward        ║')
        print('║  A/D    : Turn left / right              ║')
        print('║  SPACE  : Stand                          ║')
        print('║  Z      : Sit    X : Stretch             ║')
        print('║  1/2/3/4: Select leg FR/FL/RR/RL         ║')
        print('║  ↑↓     : Tibia up/down  (pitch)         ║')
        print('║  ←→     : Coxa left/right (yaw)          ║')
        print('║  +/-    : Increase/decrease step size    ║')
        print('║  Q      : Quit                           ║')
        print('╠══════════════════════════════════════════╣')
        print('║  Selected leg : {:3s}   Step: {:.2f} rad   ║'.format(
            LEG_NAMES[selected_leg], step))
        print('╠══════════════════════════════════════════╣')
        print('║  Joint Angles (degrees):                 ║')
        for i, (name, val) in enumerate(zip(JOINT_LABELS, joints)):
            marker = '► ' if i//2 == selected_leg else '  '
            print('║  {}{:20s}: {:+6.1f}°          ║'.format(
                marker, name, math.degrees(val)))
        print('╚══════════════════════════════════════════╝')

    node.send(POSES['stand'], 2.0)
    print_status()

    while True:
        try:
            k = get_key()
            nonlocal_step = [step]

            if k in ('q', 'Q', '\x03'):
                break
            elif k == ' ':
                joints = list(POSES['stand'])
                node.send(joints, 1.5)
            elif k == 'z':
                joints = list(POSES['sit'])
                node.send(joints, 1.5)
            elif k == 'x':
                joints = list(POSES['stretch'])
                node.send(joints, 1.5)
            elif k == 'w':
                for p in ['walk_a', 'walk_b']:
                    node.send(POSES[p], 0.5)
                    time.sleep(0.2)
                joints = list(node.current)
            elif k == 's':
                for p in ['walk_b', 'walk_a']:
                    node.send(POSES[p], 0.5)
                    time.sleep(0.2)
                joints = list(node.current)
            elif k == 'a':
                j = list(POSES['stand'])
                j[0] =  0.4; j[2] =  0.4
                j[4] = -0.4; j[6] = -0.4
                node.send(j, 0.8)
                joints = list(j)
            elif k == 'd':
                j = list(POSES['stand'])
                j[0] = -0.4; j[2] = -0.4
                j[4] =  0.4; j[6] =  0.4
                node.send(j, 0.8)
                joints = list(j)
            elif k == '1': selected_leg = 0
            elif k == '2': selected_leg = 1
            elif k == '3': selected_leg = 2
            elif k == '4': selected_leg = 3
            elif k == '+': step = min(0.3, step + 0.02)
            elif k == '-': step = max(0.02, step - 0.02)
            elif k == '\x1b[A':   # up — tibia pitch
                i = selected_leg * 2 + 1
                joints[i] = max(-1.57, joints[i] - step)
                node.send(joints, 0.4)
            elif k == '\x1b[B':   # down — tibia pitch
                i = selected_leg * 2 + 1
                joints[i] = min(1.57, joints[i] + step)
                node.send(joints, 0.4)
            elif k == '\x1b[C':   # right — coxa yaw
                i = selected_leg * 2
                joints[i] = min(1.57, joints[i] + step)
                node.send(joints, 0.4)
            elif k == '\x1b[D':   # left — coxa yaw
                i = selected_leg * 2
                joints[i] = max(-1.57, joints[i] - step)
                node.send(joints, 0.4)

            print_status()

        except KeyboardInterrupt:
            break

    node.send(POSES['stand'], 2.0)
    print('\nReturned to stand. Bye!')


# ══════════════════════════════════════════════════════════
# METHOD 3: CLI COMMAND LINE
# ══════════════════════════════════════════════════════════
def run_cli(node):
    print('\n╔══════════════════════════════════════════════╗')
    print('║     SpiderBot CLI Manual Control             ║')
    print('╠══════════════════════════════════════════════╣')
    print('║  Commands:                                   ║')
    print('║  <pose>              — named pose            ║')
    print('║  fr1 <deg>           — single joint angle    ║')
    print('║  fr1 <deg> fr2 <deg> — multiple joints       ║')
    print('║  all <deg>           — all joints same angle ║')
    print('║  coxa <deg>          — all coxa joints       ║')
    print('║  tibia <deg>         — all tibia joints      ║')
    print('║  status              — show current angles   ║')
    print('║  poses               — list all presets      ║')
    print('║  q                   — quit                  ║')
    print('╠══════════════════════════════════════════════╣')
    print('║  Joint names: fr1 fr2 fl1 fl2 rr1 rr2 rl1 rl2║')
    print('╚══════════════════════════════════════════════╝\n')

    joint_map = {name: i for i, name in enumerate(
        ['fr1','fr2','fl1','fl2','rr1','rr2','rl1','rl2']
    )}
    current = list(POSES['stand'])
    node.send(current, 2.0)

    while True:
        try:
            cmd = input('\nCommand: ').strip().lower()
            if not cmd:
                continue
            parts = cmd.split()

            if parts[0] == 'q':
                break

            elif parts[0] == 'status':
                print('\nCurrent joint angles:')
                for name, val in zip(
                    ['fr1','fr2','fl1','fl2','rr1','rr2','rl1','rl2'],
                    current
                ):
                    print('  {:4s}: {:+6.1f}°'.format(
                        name, math.degrees(val)))

            elif parts[0] == 'poses':
                print('\nAvailable poses:', list(POSES.keys()))

            elif parts[0] in POSES:
                current = list(POSES[parts[0]])
                node.send(current, 1.5)

            elif parts[0] == 'all' and len(parts) == 2:
                deg = float(parts[1])
                current = [math.radians(deg)] * 8
                node.send(current, 1.0)

            elif parts[0] == 'coxa' and len(parts) == 2:
                deg = float(parts[1])
                for i in [0, 2, 4, 6]:
                    current[i] = math.radians(deg)
                node.send(current, 1.0)

            elif parts[0] == 'tibia' and len(parts) == 2:
                deg = float(parts[1])
                for i in [1, 3, 5, 7]:
                    current[i] = math.radians(deg)
                node.send(current, 1.0)

            elif parts[0] in joint_map and len(parts) == 2:
                idx = joint_map[parts[0]]
                current[idx] = math.radians(float(parts[1]))
                node.send(current, 0.8)

            else:
                # Try parsing pairs: fr1 30 fr2 -20 ...
                try:
                    i = 0
                    changed = False
                    while i < len(parts) - 1:
                        jname = parts[i]
                        deg   = float(parts[i+1])
                        if jname in joint_map:
                            current[joint_map[jname]] = math.radians(deg)
                            changed = True
                        i += 2
                    if changed:
                        node.send(current, 0.8)
                    else:
                        print('Unknown command:', cmd)
                        print('Joints: fr1 fr2 fl1 fl2 rr1 rr2 rl1 rl2')
                        print('Poses:', list(POSES.keys()))
                except Exception:
                    print('Unknown command:', cmd)

        except KeyboardInterrupt:
            break
        except Exception as e:
            print('Error:', e)

    node.send(POSES['stand'], 2.0)
    print('Returned to stand. Bye!')


# ══════════════════════════════════════════════════════════
# MAIN
# ══════════════════════════════════════════════════════════
def main():
    rclpy.init()
    node = RobotNode()

    mode = sys.argv[1] if len(sys.argv) > 1 else 'gui'

    if mode == 'keyboard':
        spin_t = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
        spin_t.start()
        run_keyboard(node)
    elif mode == 'cli':
        spin_t = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
        spin_t.start()
        run_cli(node)
    else:
        # GUI runs in main thread, ROS in background
        spin_t = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
        spin_t.start()
        run_gui(node)

    rclpy.shutdown()


if __name__ == '__main__':
    main()
