#!/usr/bin/env python3
"""
hardware_bridge.py  —  spider_robot  (FIXED)
=============================================
Subscribes to /joint_states published by Gazebo/MoveIt and
sends 8 joint angles in DEGREES to ESP32 over USB serial.

Key fixes:
  1. Correctly maps joint names to channel order every time
  2. Sends data immediately on every /joint_states message
  3. Prints debug to confirm data is flowing
  4. Handles joint_states published in any order
"""

import math
import time
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState

try:
    import serial
    SERIAL_OK = True
except ImportError:
    SERIAL_OK = False

# ── Joint order MUST match ESP32 CH0–CH7 ──────────────────────────
JOINT_ORDER = [
    'fr1_joint',   # CH0 — FR shoulder yaw
    'fr2_joint',   # CH1 — FR knee pitch
    'fl1_joint',   # CH2 — FL shoulder yaw
    'fl2_joint',   # CH3 — FL knee pitch
    'rr1_joint',   # CH4 — RR shoulder yaw
    'rr2_joint',   # CH5 — RR knee pitch
    'rl1_joint',   # CH6 — RL shoulder yaw
    'rl2_joint',   # CH7 — RL knee pitch
]


class HardwareBridge(Node):

    def __init__(self):
        super().__init__('spider_robot_hardware_bridge')

        # ── Parameters ────────────────────────────────────────
        self.declare_parameter('port',             '/dev/ttyUSB0')
        self.declare_parameter('baud',             115200)
        self.declare_parameter('publish_rate_hz',  20.0)
        self.declare_parameter('dry_run',          False)

        self.port    = self.get_parameter('port').value
        self.baud    = self.get_parameter('baud').value
        self.dry_run = self.get_parameter('dry_run').value
        rate_hz      = self.get_parameter('publish_rate_hz').value

        self.min_interval = 1.0 / rate_hz
        self.last_sent    = 0.0
        self.pkt_count    = 0

        # ── Build a lookup: joint_name → index in JOINT_ORDER ──
        self.joint_idx = {name: i for i, name in enumerate(JOINT_ORDER)}

        # ── Current joint positions (radians) ─────────────────
        self.positions = [0.0] * 8
        self.has_data  = False

        # ── Open serial ───────────────────────────────────────
        self.ser = None
        if not self.dry_run:
            self._open_serial()
        else:
            self.get_logger().warn('DRY RUN — no serial output')

        # ── Subscribe to /joint_states ─────────────────────────
        self.sub = self.create_subscription(
            JointState,
            '/joint_states',
            self._joint_cb,
            10
        )

        self.get_logger().info(
            'Hardware bridge ready  port={} baud={} dry_run={}'.format(
                self.port, self.baud, self.dry_run))
        self.get_logger().info(
            'Joint order (CH0-CH7): {}'.format(JOINT_ORDER))
        self.get_logger().info(
            'Waiting for /joint_states topic...')

    # ── Open serial port ──────────────────────────────────────
    def _open_serial(self):
        if not SERIAL_OK:
            self.get_logger().error(
                'pyserial not installed! Run: pip3 install pyserial --user')
            self.dry_run = True
            return
        try:
            self.ser = serial.Serial(
                self.port, self.baud,
                timeout=0.1,
                write_timeout=0.5
            )
            time.sleep(0.1)
            self.get_logger().info(
                'Serial opened: {} @ {}'.format(self.port, self.baud))
        except Exception as e:
            self.get_logger().error(
                'Cannot open serial {}: {}'.format(self.port, e))
            self.get_logger().warn('Switching to DRY RUN mode')
            self.dry_run = True

    # ── Joint state callback ───────────────────────────────────
    def _joint_cb(self, msg: JointState):

        # Map every received joint into our fixed order
        updated = False
        for name, pos in zip(msg.name, msg.position):
            if name in self.joint_idx:
                self.positions[self.joint_idx[name]] = pos
                updated = True

        if not updated:
            return   # message had none of our joints — ignore

        self.has_data = True

        # Rate-limit
        now = time.monotonic()
        if (now - self.last_sent) < self.min_interval:
            return
        self.last_sent = now

        self._send()

    # ── Build CSV and send ────────────────────────────────────
    def _send(self):
        if not self.has_data:
            return

        # Convert radians → degrees, clamp to servo range
        deg_vals = []
        for rad in self.positions:
            deg = math.degrees(rad)
            deg = max(-90.0, min(90.0, deg))
            deg_vals.append(deg)

        line = ','.join('{:.2f}'.format(d) for d in deg_vals) + '\n'

        if self.dry_run:
            self.get_logger().info(
                'DRY → {}'.format(line.strip()),
                throttle_duration_sec=1.0)
            return

        try:
            self.ser.write(line.encode('ascii'))
            self.pkt_count += 1

            # Debug log every 40 packets (~2 seconds at 20Hz)
            if self.pkt_count % 40 == 0:
                self.get_logger().info(
                    'Sent #{} → {}'.format(self.pkt_count, line.strip()))

        except serial.SerialTimeoutException:
            self.get_logger().warn('Serial write timeout — ESP32 busy?')
        except Exception as e:
            self.get_logger().error(
                'Serial write error: {} — reconnecting'.format(e))
            self._open_serial()

    # ── Cleanup ───────────────────────────────────────────────
    def destroy_node(self):
        if self.ser and self.ser.is_open:
            # Send stand pose before exit
            stand = '0.00,17.19,0.00,17.19,0.00,17.19,0.00,17.19\n'
            try:
                self.ser.write(stand.encode('ascii'))
                time.sleep(0.1)
            except Exception:
                pass
            self.ser.close()
            self.get_logger().info('Serial closed.')
        super().destroy_node()


def main():
    rclpy.init()
    node = HardwareBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
