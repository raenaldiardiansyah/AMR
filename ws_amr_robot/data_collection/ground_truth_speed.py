#!/usr/bin/env python3
"""
ground_truth_speed.py
=============================================================
Resolve the measurement ambiguity with a PHYSICAL reference.

The encoder telemetry alone cannot tell us whether the robot truly reaches the
commanded speed, because that depends on the encoder scale (ticks/rev) and on
whether the firmware PID really converges, both of which we'd be "proving" with
the same encoder. A tape measure + clock is independent ground truth.

PROCEDURE:
  1. Mark a straight course on the floor with tape, measure it exactly
     (e.g. 2.000 m). Pass it as --distance.
  2. Place the robot so it can accelerate BEFORE the start line (start it
     ~0.5 m back) and reach steady speed by the start line.
  3. Run this script. It commands the speed and streams live encoder distance.
  4. Press ENTER when the robot's reference point crosses the START line
     (zeros the timer + encoder distance).
  5. Press ENTER again when it crosses the FINISH line (stops timing + motors).

It then prints, for the SAME interval:
  - commanded speed
  - REAL speed   = tape distance / elapsed time      (ground truth)
  - encoder speed = encoder distance / elapsed time   (what the robot thinks)
  - encoder vs tape distance error (tells you if ticks/rev is right)

USAGE (ROS killed, clear straight space):
  python3 ground_truth_speed.py --speed 150 --distance 2.0 --set-gains \
      --kp 8 --ki 5 --kd 0
=============================================================
"""

import os
import sys
import time
import threading
import argparse

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import amr_test_utils as U
from collect_pid_step_response import (
    MM_PER_TICK, connect_serial, send_velocity,
    drain_and_get_latest, read_all_packets, tick_delta, set_gains_live,
)


def main():
    p = argparse.ArgumentParser(description='Physical ground-truth speed check')
    p.add_argument('--speed', type=int, default=150, help='commanded speed mm/s')
    p.add_argument('--distance', type=float, required=True,
                   help='tape-measured course length in METERS')
    p.add_argument('--kp', type=float, default=8.0)
    p.add_argument('--ki', type=float, default=5.0)
    p.add_argument('--kd', type=float, default=0.0)
    p.add_argument('--set-gains', action='store_true')
    args = p.parse_args()

    print("=" * 60)
    print(f"GROUND-TRUTH SPEED CHECK")
    print(f"Commanded : {args.speed} mm/s ({args.speed/1000:.3f} m/s)")
    print(f"Course    : {args.distance:.3f} m (tape)")
    print("Robot must reach steady speed BEFORE the start line.")
    print("=" * 60)

    ser = connect_serial()
    if args.set_gains:
        if not set_gains_live(ser, args.kp, args.ki, args.kd):
            print("Gains not confirmed; aborting.")
            ser.close()
            return

    # baseline tick reference
    first = None
    deadline = time.time() + 1.5
    while time.time() < deadline:
        send_velocity(ser, 0)
        d = drain_and_get_latest(ser)
        if d is not None and 'l' in d and 'r' in d:
            first = d
            break
        time.sleep(0.05)
    if first is None:
        print("ERROR: no telemetry."); ser.close(); return

    # shared state updated by the serial thread
    state = {'cum_l': 0, 'cum_r': 0, 'prev': first, 'run': True,
             'mark_l': 0, 'mark_r': 0, 't_mark': None}

    def serial_loop():
        while state['run']:
            send_velocity(ser, args.speed)
            time.sleep(0.01)
            for pkt in read_all_packets(ser):
                if 'l' not in pkt or 'r' not in pkt:
                    continue
                state['cum_l'] += tick_delta(pkt['l'], state['prev']['l'])
                state['cum_r'] += -tick_delta(pkt['r'], state['prev']['r'])
                state['prev'] = pkt

    th = threading.Thread(target=serial_loop, daemon=True)
    th.start()

    try:
        input("\nGet the robot moving, then press ENTER at the START line...")
        state['mark_l'] = state['cum_l']
        state['mark_r'] = state['cum_r']
        state['t_mark'] = time.time()
        print("  >>> START. Press ENTER at the FINISH line...")
        input()
        t_end = time.time()
        ml, mr = state['cum_l'], state['cum_r']
    finally:
        state['run'] = False
        time.sleep(0.1)
        for _ in range(10):
            send_velocity(ser, 0)
            time.sleep(0.03)
        ser.close()

    elapsed = t_end - state['t_mark']
    dl = (ml - state['mark_l']) * MM_PER_TICK / 1000.0   # m
    dr = (mr - state['mark_r']) * MM_PER_TICK / 1000.0   # m
    enc_dist = (dl + dr) / 2.0
    real_speed = args.distance / elapsed
    enc_speed = enc_dist / elapsed

    print("\n" + "=" * 60)
    print("RESULT")
    print("=" * 60)
    print(f"  elapsed time      : {elapsed:.2f} s")
    print(f"  commanded speed   : {args.speed/1000:.3f} m/s")
    print(f"  REAL speed (tape) : {real_speed:.3f} m/s   <-- ground truth")
    print(f"  encoder speed     : {enc_speed:.3f} m/s")
    print(f"  encoder distance  : {enc_dist:.3f} m   (L={dl:.3f}, R={dr:.3f})")
    print(f"  tape distance     : {args.distance:.3f} m")
    de = (enc_dist - args.distance) / args.distance * 100
    print(f"  encoder vs tape   : {de:+.1f}%   (scale check; ~0% = 2400 is right)")
    print()
    rc = (real_speed - args.speed/1000) / (args.speed/1000) * 100
    print(f"  real vs commanded : {rc:+.1f}%")
    if abs(rc) < 8:
        print("  --> PID REACHES target. The ~99 (old) method was right.")
    elif rc < -20:
        print("  --> Robot UNDERSHOOTS badly. The ~67 (new) method was right;")
        print("      real steady-state error is large -> retune / raise Ki.")
    else:
        print("  --> Partial undershoot; see the numbers.")
    if abs(de) > 8:
        print(f"  --> Encoder distance off by {de:+.0f}% -> ticks/rev (2400) is WRONG,")
        print("      that is the real bug; recalibrate scale before trusting speeds.")


if __name__ == '__main__':
    main()
