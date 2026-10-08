#!/usr/bin/env python3
"""
encoder_ticks_per_rev_calibration.py
------------------------------------
Calibrate encoder scale by PUSHING the robot a fixed distance several times
and averaging the tick count. Pushing keeps the motors out of the loop (pure
encoder measurement, no PWM/drift), and repeating reduces per-push error.

Per push you move the robot a fixed distance (default 0.80 m) along a straight
line. Do it 5 to 10 times. The script averages the tick delta and computes,
per wheel:
  meters_per_tick = push_distance / avg_ticks_per_push   <- what odometry needs
  ticks_per_rev   = avg_ticks * pi * wheel_d / push_dist  <- to compare with 2400

It also reports the LEFT/RIGHT mismatch. Because you push straight along a line
(use a wall or floor tape as a guide), this mismatch is trustworthy and is a
prime suspect for a constant left/right drift.

HOW TO USE:
  Terminal 1: ~/kill_robot.sh && ros2 launch robot_bringup hardware_launch.py
  Terminal 2:
    source ~/Autonomous-Mobile-Robot/ws_amr_robot/install/setup.bash
    python3 .../characterization/encoder_ticks_per_rev_calibration.py

  Set up a straight guide line and mark 0.80 m steps on the floor. For each
  push: capture START (ENTER), push exactly one step, capture END (ENTER).
  Repeat 5-10 times, then type 'q' to finish and see the averages.

NOTE: /wheel_encoders is Int32MultiArray [left_ticks, right_ticks]. The right
encoder reads negative going forward, so we use the ABSOLUTE delta.
"""

import math
import statistics
import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32MultiArray

CURRENT_TICKS_PER_REV = 2400.0   # currently assumed, for comparison
WHEEL_DIAMETER = 0.150           # m, ban 15 cm (robot besar)
DEFAULT_PUSH_M = 0.80            # meters per push


class EncoderReader(Node):
    def __init__(self):
        super().__init__('encoder_ticks_per_rev_calibration')
        self.left = None
        self.right = None
        self.create_subscription(Int32MultiArray, '/wheel_encoders',
                                 self._cb, 10)

    def _cb(self, msg):
        if len(msg.data) >= 2:
            self.left = msg.data[0]
            self.right = msg.data[1]

    def capture(self, settle_spins=20):
        for _ in range(settle_spins):
            rclpy.spin_once(self, timeout_sec=0.05)
        return self.left, self.right


def get_float(prompt, default):
    raw = input(f'{prompt} [{default}]: ').strip()
    if raw == '':
        return default
    try:
        v = float(raw)
        return v if v > 0 else default
    except ValueError:
        return default


def main():
    rclpy.init()
    node = EncoderReader()

    print()
    print('=' * 60)
    print('  ENCODER SCALE CALIBRATION (push a fixed distance, averaged)')
    print('=' * 60)
    print('  Hardware launch must be running so /wheel_encoders publishes.')
    print('  Motors stay off; you push the robot by hand.')
    print()
    print('  Waiting for /wheel_encoders...')
    t = 0
    while node.left is None and t < 100:
        rclpy.spin_once(node, timeout_sec=0.1)
        t += 1
    if node.left is None:
        print('  No data on /wheel_encoders. Is the hardware launch running?')
        node.destroy_node()
        rclpy.shutdown()
        return
    print(f'  Connected. Current counts: left={node.left} right={node.right}')

    push_m = get_float('  Distance per push (meters)', DEFAULT_PUSH_M)
    print(f'  Each push = {push_m:.2f} m. Mark steps on the floor along a straight line.')
    print()

    left_deltas = []
    right_deltas = []
    n = 0
    while True:
        cmd = input(f'  Push #{n+1}: ENTER to capture START, or "q" to finish: ').strip().lower()
        if cmd == 'q':
            break
        l0, r0 = node.capture()
        input(f'    Push the robot exactly {push_m:.2f} m, then press ENTER...')
        l1, r1 = node.capture()
        dl = abs(l1 - l0)
        dr = abs(r1 - r0)
        if dl == 0 and dr == 0:
            print('    Both wheels counted 0 ticks. Skipped (did it move?).')
            continue
        left_deltas.append(dl)
        right_deltas.append(dr)
        n += 1
        print(f'    push #{n}: left={dl} ticks  right={dr} ticks')

    if n == 0:
        print('  No valid pushes recorded. Nothing to compute.')
        node.destroy_node()
        rclpy.shutdown()
        return

    avg_l = statistics.mean(left_deltas)
    avg_r = statistics.mean(right_deltas)
    sd_l = statistics.pstdev(left_deltas) if n > 1 else 0.0
    sd_r = statistics.pstdev(right_deltas) if n > 1 else 0.0

    mpt_l = push_m / avg_l
    mpt_r = push_m / avg_r
    tpr_l = avg_l * math.pi * WHEEL_DIAMETER / push_m
    tpr_r = avg_r * math.pi * WHEEL_DIAMETER / push_m

    print()
    print('=' * 60)
    print(f'  RESULTS  ({n} pushes of {push_m:.2f} m each)')
    print('=' * 60)
    print(f'  LEFT  ticks/push: avg {avg_l:.1f}  (spread +/-{sd_l:.1f})')
    print(f'  RIGHT ticks/push: avg {avg_r:.1f}  (spread +/-{sd_r:.1f})')
    print()
    print(f'  LEFT  meters_per_tick = {mpt_l:.8f}   ticks_per_rev = {tpr_l:.1f}')
    print(f'  RIGHT meters_per_tick = {mpt_r:.8f}   ticks_per_rev = {tpr_r:.1f}')
    print(f'  (currently assumed ticks_per_rev = {CURRENT_TICKS_PER_REV:.0f})')

    # a large spread means inconsistent pushes -> distances were not equal
    for name, sd, avg in (('left', sd_l, avg_l), ('right', sd_r, avg_r)):
        if avg > 0 and sd / avg > 0.05:
            print(f'  WARNING: {name} push spread is >5%. Your pushes were not')
            print('           equal distances. Re-do with a marked floor guide.')

    mismatch = (avg_r - avg_l) / avg_l * 100.0
    print()
    print(f'  LEFT/RIGHT tick mismatch = {mismatch:+.1f}%')
    if abs(mismatch) > 2.0:
        print('  >> Wheels count differently for the same travel. This is a real')
        print('     encoder-scale mismatch and a likely cause of your drift.')
    else:
        print('  >> Left and right agree closely. Encoder scale is NOT your drift')
        print('     cause; look at motor strength / PWM saturation instead.')
    avg_tpr = (tpr_l + tpr_r) / 2.0
    print(f'  average ticks_per_rev = {avg_tpr:.1f}  (use if you keep one value)')
    print()
    print('  Tell me these numbers and I will point to the exact lines to update')
    print('  in odometry_node.py and any script with TICKS_PER_REV.')

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
