#!/usr/bin/env python3
"""
ramp_speed_characterization.py
------------------------------
Find the REAL speed limits of the robot by ramping the command up in steps
and reading every sensor at each step, until the robot stops getting faster.

This tells you the values to put in custom_controller.yaml:
  - max_linear_speed  : where forward speed saturates (command rises, speed does not)
  - min_linear_speed  : the lowest command that still produces motion (deadband)
  - max_angular_speed : where rotation speed saturates
  - min_angular_speed : the lowest command that still rotates (rotation deadband)

It is also a PID diagnostic: if measured speed does NOT track the command
(big lag, nonlinear, saturates far below the command), the STM32 PID needs
work BEFORE you trust these limits. If it tracks cleanly, the PID is fine.

SENSORS READ AT EACH STEP (so you can cross-check them):
  /odom          EKF fused velocity   (twist.linear.x / angular.z)
  /odom_raw      wheel-encoder odom   (twist.linear.x / angular.z)
  /odom_rf2o     laser odometry       (twist.linear.x / angular.z)
  /imu/data_raw  gyro                 (angular_velocity.z, best for rotation)
  /wheel_encoders raw ticks           (converted to per-wheel mm/s)

HOW TO USE:
  Terminal 1: ~/kill_robot.sh && ros2 launch robot_bringup hardware_launch.py
  Terminal 2:
    source ~/Autonomous-Mobile-Robot/ws_amr_robot/install/setup.bash
    # rotation test (safe, robot stays in place):
    python3 .../characterization/ramp_speed_characterization.py angular
    # forward test (NEEDS a long clear aisle, robot drives forward):
    python3 .../characterization/ramp_speed_characterization.py linear

OUTPUT: ~/thesis_data/PID_tune_STM32/ramp_<mode>_<timestamp>.csv
Keep your hand on the emergency stop. New limits on real motors.
"""

import sys
import time
import math
import os
from datetime import datetime

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
from std_msgs.msg import Int32MultiArray

# ---- ramp settings (edit these to change the sweep) ----
LINEAR = {
    'start': 0.02, 'stop': 1.0, 'step': 0.02,   # m/s
    'dwell': 1.2,   # seconds held at each level
    'unit': 'm/s', 'label': 'linear.x',
}
ANGULAR = {
    'start': 0.10, 'stop': 1.20, 'step': 0.10,   # rad/s
    'dwell': 1.5,
    'unit': 'rad/s', 'label': 'angular.z',
}
SETTLE_FRACTION = 0.5    # ignore the first half of each dwell (let it settle)
MOTION_THRESHOLD = 0.01  # measured speed above this counts as "moving" (deadband)
SATURATION_EPS = 0.005   # measured rise smaller than this counts as "plateau"

# encoder geometry (from project memory)
TICKS_PER_REV = 2400.0
WHEEL_DIAMETER = 0.150   # m
M_PER_TICK = (math.pi * WHEEL_DIAMETER) / TICKS_PER_REV


class RampCharacterizer(Node):
    def __init__(self):
        super().__init__('ramp_speed_characterization')
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        # latest instantaneous readings
        self.odom_v = self.odom_w = None
        self.enc_v = self.enc_w = None
        self.rf2o_v = self.rf2o_w = None
        self.imu_w = None
        self.enc_ticks = None   # (left, right) cumulative

        self.create_subscription(Odometry, '/odom', self._cb_odom, 10)
        self.create_subscription(Odometry, '/odom_raw', self._cb_enc, 10)
        self.create_subscription(Odometry, '/odom_rf2o', self._cb_rf2o, 10)
        self.create_subscription(Imu, '/imu/data_raw', self._cb_imu, 10)
        self.create_subscription(Int32MultiArray, '/wheel_encoders',
                                 self._cb_ticks, 10)

    def _cb_odom(self, m):
        self.odom_v = m.twist.twist.linear.x
        self.odom_w = m.twist.twist.angular.z

    def _cb_enc(self, m):
        self.enc_v = m.twist.twist.linear.x
        self.enc_w = m.twist.twist.angular.z

    def _cb_rf2o(self, m):
        self.rf2o_v = m.twist.twist.linear.x
        self.rf2o_w = m.twist.twist.angular.z

    def _cb_imu(self, m):
        self.imu_w = m.angular_velocity.z

    def _cb_ticks(self, m):
        if len(m.data) >= 2:
            self.enc_ticks = (m.data[0], m.data[1])

    def send(self, mode, value):
        msg = Twist()
        if mode == 'linear':
            msg.linear.x = value
        else:
            msg.angular.z = value
        self.cmd_pub.publish(msg)

    def stop(self):
        for _ in range(8):
            self.cmd_pub.publish(Twist())
            rclpy.spin_once(self, timeout_sec=0.05)

    def measure_level(self, mode, value, dwell):
        """Hold the command for `dwell` seconds, average the sensors over the
        settled part of the window, return a dict of measured values."""
        settle_until = time.time() + dwell * SETTLE_FRACTION
        end = time.time() + dwell

        # collect samples only after the settle window
        samples = {'odom': [], 'enc': [], 'rf2o': [], 'imu': []}
        tick0 = t0 = None
        tick1 = t1 = None

        while time.time() < end:
            self.send(mode, value)
            rclpy.spin_once(self, timeout_sec=0.05)
            if time.time() < settle_until:
                continue
            key_v = 'angular' if mode == 'angular' else 'linear'
            # pick the right axis per sensor
            o = self.odom_w if mode == 'angular' else self.odom_v
            e = self.enc_w if mode == 'angular' else self.enc_v
            r = self.rf2o_w if mode == 'angular' else self.rf2o_v
            if o is not None: samples['odom'].append(o)
            if e is not None: samples['enc'].append(e)
            if r is not None: samples['rf2o'].append(r)
            if self.imu_w is not None: samples['imu'].append(self.imu_w)
            # wheel-tick rate across the settled window
            if self.enc_ticks is not None:
                if tick0 is None:
                    tick0, t0 = self.enc_ticks, time.time()
                tick1, t1 = self.enc_ticks, time.time()

        def avg(lst):
            return sum(lst) / len(lst) if lst else float('nan')

        # per-wheel speed from raw ticks (magnitude; right polarity is negative
        # forward on this robot, so we report absolute wheel speed)
        wl = wr = float('nan')
        if tick0 is not None and tick1 is not None and t1 > t0:
            dt = t1 - t0
            wl = abs((tick1[0] - tick0[0]) * M_PER_TICK / dt)
            wr = abs((tick1[1] - tick0[1]) * M_PER_TICK / dt)

        return {
            'cmd': value,
            'odom': avg(samples['odom']),
            'enc': avg(samples['enc']),
            'rf2o': avg(samples['rf2o']),
            'imu': avg(samples['imu']),
            'wheel_l': wl, 'wheel_r': wr,
        }


def main():
    mode = sys.argv[1].lower() if len(sys.argv) > 1 else 'angular'
    if mode not in ('linear', 'angular'):
        print("  Usage: ramp_speed_characterization.py [linear|angular]")
        return
    cfg = LINEAR if mode == 'linear' else ANGULAR

    rclpy.init()
    node = RampCharacterizer()

    out_dir = os.path.expanduser('~/thesis_data/PID_tune_STM32')
    os.makedirs(out_dir, exist_ok=True)
    out_path = os.path.join(
        out_dir, f"ramp_{mode}_{datetime.now().strftime('%Y%m%d_%H%M%S')}.csv")

    print()
    print('=' * 64)
    print(f'  RAMP CHARACTERIZATION - {mode.upper()}  ({cfg["unit"]})')
    print('=' * 64)
    if mode == 'linear':
        print('  WARNING: the robot DRIVES FORWARD continuously during this test.')
        print('  Use a long clear aisle. Hand on the emergency stop.')
    else:
        print('  The robot ROTATES IN PLACE. Keep the area around it clear.')
    print(f'  Sweep: {cfg["start"]} -> {cfg["stop"]} step {cfg["step"]}, '
          f'{cfg["dwell"]}s each')
    print()
    input('  Press ENTER to begin...')

    # wait for topics
    print('  Waiting for sensor topics...')
    t = time.time()
    while time.time() - t < 5.0:
        rclpy.spin_once(node, timeout_sec=0.1)

    import csv
    f = open(out_path, 'w', newline='')
    w = csv.writer(f)
    w.writerow(['cmd_' + cfg['label'], 'odom', 'enc', 'rf2o', 'imu_gyro',
                'wheel_l_mps', 'wheel_r_mps'])

    rows = []
    print()
    print(f'  {"cmd":>7} | {"odom":>7} {"enc":>7} {"rf2o":>7} {"imu":>7} | '
          f'{"whL":>6} {"whR":>6}')
    print('  ' + '-' * 60)

    value = cfg['start']
    prev_meas = 0.0
    plateau_count = 0
    try:
        while value <= cfg['stop'] + 1e-9:
            res = node.measure_level(mode, value, cfg['dwell'])
            w.writerow([f'{value:.3f}',
                        f'{res["odom"]:.4f}', f'{res["enc"]:.4f}',
                        f'{res["rf2o"]:.4f}', f'{res["imu"]:.4f}',
                        f'{res["wheel_l"]:.4f}', f'{res["wheel_r"]:.4f}'])
            f.flush()
            rows.append(res)
            print(f'  {value:7.3f} | {res["odom"]:7.3f} {res["enc"]:7.3f} '
                  f'{res["rf2o"]:7.3f} {res["imu"]:7.3f} | '
                  f'{res["wheel_l"]:6.3f} {res["wheel_r"]:6.3f}')

            # saturation watch on the EKF reading (the controller's reference)
            meas = abs(res['odom']) if not math.isnan(res['odom']) else 0.0
            if meas - prev_meas < SATURATION_EPS:
                plateau_count += 1
            else:
                plateau_count = 0
            prev_meas = meas
            value += cfg['step']
    except KeyboardInterrupt:
        print('\n  Interrupted by user.')
    finally:
        node.stop()
        f.close()

    # ---- summary: deadband and saturation ----
    print()
    print('  ' + '=' * 60)
    print('  RESULTS (based on the EKF /odom reading)')
    print('  ' + '=' * 60)

    moving = [r for r in rows
              if not math.isnan(r['odom']) and abs(r['odom']) > MOTION_THRESHOLD]
    if moving:
        deadband = moving[0]['cmd']
        peak = max(rows, key=lambda r: 0 if math.isnan(r['odom']) else abs(r['odom']))
        print(f'  Deadband (lowest command that moved): {deadband:.3f} {cfg["unit"]}')
        print(f'  Peak measured speed: {abs(peak["odom"]):.3f} {cfg["unit"]} '
              f'at command {peak["cmd"]:.3f}')
        if mode == 'linear':
            print(f'  -> set min_linear_speed ~ {deadband:.2f}, '
                  f'max_linear_speed ~ {abs(peak["odom"]):.2f}')
        else:
            print(f'  -> set min_angular_speed ~ {deadband:.2f}, '
                  f'max_angular_speed ~ {abs(peak["odom"]):.2f}')
        # PID sanity: did measured roughly track command at the top?
        gap = peak['cmd'] - abs(peak['odom'])
        if gap > 0.25 * peak['cmd']:
            print(f'  NOTE: at the top, command {peak["cmd"]:.2f} but only '
                  f'reached {abs(peak["odom"]):.2f}. Measured does NOT track')
            print('        command well -> check the STM32 PID before trusting these.')
        else:
            print('  Measured tracks command reasonably -> PID looks OK.')
    else:
        print('  Robot never exceeded the motion threshold. Check that it')
        print('  actually moved and that the topics were publishing.')

    print()
    print(f'  Data saved to: {out_path}')
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
