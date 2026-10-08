#!/usr/bin/env python3
"""
STEP RESPONSE DATA COLLECTOR (v2 - with per-wheel encoder readings)
==============================
Thesis: Kendali dan Navigasi AMR Berbasis LiDAR dan IMU di Warehouse
Author: Reinald Mariel - Universitas Telkom

PURPOSE:
  Records the step response from speed zero to reference speed.
  Captures both the odom_raw (EKF-fused) speed AND the raw encoder
  readings (left + right wheel separately) so your professor can see
  exactly what the encoder is reporting at each moment.

HOW TO USE:
  Terminal 1:
    ~/kill_robot.sh && ros2 launch robot_bringup hardware_launch.py

  Terminal 2:
    source ~/Autonomous-Mobile-Robot/ws_amr_robot/install/setup.bash
    python3 ~/Autonomous-Mobile-Robot/ws_amr_robot/src/robot_bringup/scripts/characterization/step_response_collector.py

  Change TARGET_SPEED_MMPS below to test different speeds.
  Recommended: run at 100, 150, 200 mm/s separately.

OUTPUT CSV columns:
  time_s              - timestamp in seconds
  cmd_vel_mmps        - commanded speed (0 before step, TARGET after)
  odom_raw_speed_mmps - average robot speed from /odom_raw (EKF-fused)
  enc_left_ticks      - cumulative left encoder tick count (from STM32)
  enc_right_ticks     - cumulative right encoder tick count (from STM32)
  dt_ms               - actual STM32 telemetry interval in milliseconds
  left_speed_enc_mmps - left wheel speed computed from encoder delta/dt
  right_speed_enc_mmps- right wheel speed computed from encoder delta/dt

SEND THIS CSV TO YOUR PROFESSOR for step response analysis.
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from std_msgs.msg import Int32MultiArray
import csv
import math
import os
import time
import datetime

# ============================================================
# PHYSICAL CONSTANTS
# ============================================================
WHEEL_DIAMETER = 0.150        # meters
TICKS_PER_REV  = 2400.0
MM_PER_TICK    = (math.pi * WHEEL_DIAMETER) / TICKS_PER_REV * 1000.0
POLARITY_LEFT  =  1.0         # left encoder: positive ticks = forward
POLARITY_RIGHT = -1.0         # right encoder: negative ticks = forward

# ============================================================
# SETTINGS - change TARGET_SPEED_MMPS between runs
# ============================================================
TARGET_SPEED_MMPS  = 150      # mm/s. Try 100, 150, 200.
HOLD_DURATION      = 5.0      # seconds to hold at target speed
PRE_STEP_DURATION  = 2.0      # seconds at 0 before the step (baseline)
POST_STEP_DURATION = 2.0      # seconds at 0 after the step (coast down)
SAMPLE_RATE_HZ     = 50       # samples per second

OUTPUT_DIR = os.path.expanduser('~/thesis_data/step_response')


class StepResponseCollector(Node):
    def __init__(self):
        super().__init__('step_response_collector')

        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        # --- State from /odom_raw ---
        self.odom_speed_mmps = 0.0
        self.sub_odom = self.create_subscription(
            Odometry, '/odom_raw', self._cb_odom, 10)

        # --- State from /wheel_encoders ---
        self.enc_left_ticks       = 0
        self.enc_right_ticks      = 0
        self.enc_dt_ms            = 50
        self.left_speed_enc_mmps  = 0.0
        self.right_speed_enc_mmps = 0.0
        self._prev_enc_left       = None
        self._prev_enc_right      = None

        self.sub_enc = self.create_subscription(
            Int32MultiArray, '/wheel_encoders', self._cb_enc, 10)

        self.get_logger().info('Step Response Collector v2 ready.')

    def _cb_odom(self, msg):
        self.odom_speed_mmps = msg.twist.twist.linear.x * 1000.0

    def _cb_enc(self, msg):
        if len(msg.data) < 2:
            return

        cur_left  = msg.data[0]
        cur_right = msg.data[1]
        dt_ms     = int(msg.data[2]) if len(msg.data) >= 3 else 50

        self.enc_left_ticks  = cur_left
        self.enc_right_ticks = cur_right
        self.enc_dt_ms       = dt_ms

        if self._prev_enc_left is not None and dt_ms > 0:
            dt_s = dt_ms / 1000.0
            delta_left  = (cur_left  - self._prev_enc_left)  * POLARITY_LEFT
            delta_right = (cur_right - self._prev_enc_right) * POLARITY_RIGHT
            self.left_speed_enc_mmps  = (delta_left  * MM_PER_TICK) / dt_s
            self.right_speed_enc_mmps = (delta_right * MM_PER_TICK) / dt_s

        self._prev_enc_left  = cur_left
        self._prev_enc_right = cur_right

    def send_speed(self, speed_mmps):
        msg = Twist()
        msg.linear.x = speed_mmps / 1000.0
        msg.angular.z = 0.0
        self.cmd_pub.publish(msg)

    def stop(self):
        self.send_speed(0)


def spin_briefly(node, duration_sec):
    end = time.time() + duration_sec
    while time.time() < end:
        rclpy.spin_once(node, timeout_sec=0.01)


def main():
    rclpy.init()
    node = StepResponseCollector()

    os.makedirs(OUTPUT_DIR, exist_ok=True)
    timestamp_str = datetime.datetime.now().strftime('%Y%m%d_%H%M%S')
    filename  = f'step_response_{TARGET_SPEED_MMPS}mmps_{timestamp_str}.csv'
    filepath  = os.path.join(OUTPUT_DIR, filename)

    print('=' * 60)
    print('  STEP RESPONSE COLLECTOR v2 (with encoder readings)')
    print('=' * 60)
    print(f'  Target speed    : {TARGET_SPEED_MMPS} mm/s')
    print(f'  Hold duration   : {HOLD_DURATION} s')
    print(f'  Output file     : {filepath}')
    print()
    print('  MAKE SURE:')
    print('  - Robot is on flat ground with space to drive forward')
    print('  - Only hardware_launch.py is running (NO navigation stack)')
    print()
    input('  Press ENTER to start...')
    print()

    sample_interval = 1.0 / SAMPLE_RATE_HZ
    records = []
    start_time = time.time()

    def snapshot(cmd_mmps):
        return [
            round(time.time() - start_time, 4),
            cmd_mmps,
            round(node.odom_speed_mmps, 2),
            node.enc_left_ticks,
            node.enc_right_ticks,
            node.enc_dt_ms,
            round(node.left_speed_enc_mmps, 2),
            round(node.right_speed_enc_mmps, 2),
        ]

    # --- Phase 1: Pre-step baseline at 0 ---
    print(f'  [1/3] Pre-step : 0 mm/s for {PRE_STEP_DURATION}s...')
    phase_end = time.time() + PRE_STEP_DURATION
    while time.time() < phase_end:
        rclpy.spin_once(node, timeout_sec=0.001)
        node.send_speed(0)
        records.append(snapshot(0))
        time.sleep(sample_interval)

    # --- Phase 2: Step to TARGET_SPEED_MMPS ---
    print(f'  [2/3] Step     : {TARGET_SPEED_MMPS} mm/s for {HOLD_DURATION}s...')
    phase_end = time.time() + HOLD_DURATION
    while time.time() < phase_end:
        rclpy.spin_once(node, timeout_sec=0.001)
        node.send_speed(TARGET_SPEED_MMPS)
        records.append(snapshot(TARGET_SPEED_MMPS))
        time.sleep(sample_interval)

    # --- Phase 3: Post-step back to 0 ---
    print(f'  [3/3] Post-step: 0 mm/s for {POST_STEP_DURATION}s...')
    phase_end = time.time() + POST_STEP_DURATION
    while time.time() < phase_end:
        rclpy.spin_once(node, timeout_sec=0.001)
        node.send_speed(0)
        records.append(snapshot(0))
        time.sleep(sample_interval)

    node.stop()
    spin_briefly(node, 0.5)

    # --- Save CSV ---
    header = [
        'time_s', 'cmd_vel_mmps',
        'odom_raw_speed_mmps',
        'enc_left_ticks', 'enc_right_ticks', 'dt_ms',
        'left_speed_enc_mmps', 'right_speed_enc_mmps',
    ]
    with open(filepath, 'w', newline='') as f:
        writer = csv.writer(f)
        writer.writerow(header)
        writer.writerows(records)

    # --- Summary stats ---
    step_records = [r for r in records if r[1] == TARGET_SPEED_MMPS]
    print()
    print('=' * 60)
    print('  DONE. Data saved.')
    print(f'  File: {filepath}')
    print(f'  Total samples: {len(records)}')

    if step_records:
        ss = step_records[int(len(step_records) * 0.4):]   # last 60% = steady state
        avg_odom  = sum(r[2] for r in ss) / len(ss)
        avg_left  = sum(r[6] for r in ss) / len(ss)
        avg_right = sum(r[7] for r in ss) / len(ss)
        avg_dt    = sum(r[5] for r in ss) / len(ss)
        err_odom  = abs(TARGET_SPEED_MMPS - avg_odom)  / TARGET_SPEED_MMPS * 100
        err_left  = abs(TARGET_SPEED_MMPS - avg_left)  / TARGET_SPEED_MMPS * 100
        err_right = abs(TARGET_SPEED_MMPS - avg_right) / TARGET_SPEED_MMPS * 100

        # Rise time: first time odom_raw reaches 90% of target
        threshold = TARGET_SPEED_MMPS * 0.9
        step_start_t = step_records[0][0]
        rise_time = None
        for r in step_records:
            if r[2] >= threshold:
                rise_time = r[0] - step_start_t
                break

        print()
        print(f'  Steady-state summary (last 60% of step phase):')
        print(f'  {"Target speed":25s}: {TARGET_SPEED_MMPS} mm/s')
        print(f'  {"odom_raw avg":25s}: {avg_odom:.1f} mm/s  (error {err_odom:.1f}%)')
        print(f'  {"Left wheel enc avg":25s}: {avg_left:.1f} mm/s  (error {err_left:.1f}%)')
        print(f'  {"Right wheel enc avg":25s}: {avg_right:.1f} mm/s  (error {err_right:.1f}%)')
        print(f'  {"Avg STM32 dt":25s}: {avg_dt:.1f} ms')
        if rise_time is not None:
            print(f'  {"Rise time (to 90%)":25s}: {rise_time:.3f} s')
        else:
            print(f'  {"Rise time":25s}: did not reach 90% of target')

        asymmetry = abs(avg_left - avg_right)
        asymmetry_pct = asymmetry / TARGET_SPEED_MMPS * 100
        print()
        print(f'  Wheel asymmetry: {asymmetry:.1f} mm/s ({asymmetry_pct:.1f}%)', end='')
        if asymmetry_pct > 10:
            print('  <-- LARGE, robot will drift sideways')
        else:
            print('  <-- OK')

    print()
    print('=' * 60)
    print('  SEND THIS FILE TO YOUR PROFESSOR:')
    print(f'  {filepath}')
    print('=' * 60)
    print()

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
