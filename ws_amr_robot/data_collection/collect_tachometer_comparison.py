#!/usr/bin/env python3
"""
collect_tachometer_comparison.py  --  Encoder vs Tachometer validation
----------------------------------------------------------------------
Inject a PWM value, let the motor reach steady state, read encoder RPM,
then you type the tachometer RPM reading. Repeat for multiple PWM values.

Validates whether the encoder reads the true motor speed or has noise/error.

USAGE:
  Kill ROS first, then:
  python3 collect_tachometer_comparison.py

  The script will ask you for PWM values one at a time.
  For each PWM:
    1. Motor spins up and holds for a few seconds
    2. Encoder RPM is displayed live
    3. You read the tachometer and type the RPM
    4. Motor stops, next PWM

  Type 'done' to finish and see the summary.

REQUIREMENTS:
  - Wheels elevated (off ground)
  - ROS stack NOT running (serial port conflict)
  - Tachometer ready
"""

import serial
import json
import math
import os
import sys
import time
from datetime import datetime

SERIAL_PORT = '/dev/serial/by-id/usb-FTDI_FT232R_USB_UART_A5069RR4-if00-port0'
BAUD_RATE = 115200
WHEEL_DIAM_MM = 150.0
TICKS_PER_REV = 2400.0
MM_PER_TICK = (math.pi * WHEEL_DIAM_MM) / TICKS_PER_REV

SETTLE_TIME = 5.0
MEASURE_TIME = 20.0

sys.path.insert(0, os.path.dirname(__file__))
import amr_test_utils as U


def connect():
    print(f"Connecting to {SERIAL_PORT}...")
    ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=0.15)
    ser.reset_input_buffer()
    time.sleep(0.6)
    ser.reset_input_buffer()
    print("  Connected.")
    return ser


def send_pwm(ser, left, right):
    cmd = f"P:{max(0,min(999,int(left)))},{max(0,min(999,int(right)))}\r\n"
    ser.write(cmd.encode())


def stop(ser):
    for _ in range(10):
        send_pwm(ser, 0, 0)
        time.sleep(0.05)


def wrap16(d):
    d %= 65536
    if d >= 32768:
        d -= 65536
    return d


def read_packets(ser):
    packets = []
    while ser.in_waiting > 0:
        try:
            line = ser.readline().decode('utf-8', errors='ignore').strip()
            if line.startswith('{') and line.endswith('}'):
                p = json.loads(line)
                if 'l' in p and 'r' in p:
                    packets.append(p)
        except Exception:
            pass
    return packets


def to_rpm(mmps):
    return (mmps / 1000.0) * 60.0 / (math.pi * WHEEL_DIAM_MM / 1000.0)


def measure_single_wheel(ser, pwm, wheel, settle_s, measure_s):
    """Run ONE motor at PWM. Returns (rpm_mean, rpm_std, speeds_list)."""
    if wheel == 'left':
        l_pwm, r_pwm = pwm, 0
    else:
        l_pwm, r_pwm = 0, pwm

    # Still phase (2s) - record zero baseline
    print(f"  Recording still baseline (2s)...")
    send_pwm(ser, 0, 0)
    ser.reset_input_buffer()
    time.sleep(0.3)
    ser.reset_input_buffer()
    prev_pkt = None
    still_speeds = []
    t_end = time.time() + 2.0
    while time.time() < t_end:
        send_pwm(ser, 0, 0)
        time.sleep(0.02)
        for pkt in read_packets(ser):
            if prev_pkt is not None:
                if wheel == 'left':
                    delta = wrap16(pkt['l'] - prev_pkt['l'])
                else:
                    delta = -wrap16(pkt['r'] - prev_pkt['r'])
                spd = delta * MM_PER_TICK / 0.050
                still_speeds.append(spd)
            prev_pkt = pkt
    still_mean = sum(still_speeds) / len(still_speeds) if still_speeds else 0
    print(f"    Still baseline: {to_rpm(still_mean):.1f} RPM ({len(still_speeds)} samples)")

    # Settle phase
    send_pwm(ser, l_pwm, r_pwm)
    ser.reset_input_buffer()
    t_end = time.time() + settle_s
    while time.time() < t_end:
        send_pwm(ser, l_pwm, r_pwm)
        read_packets(ser)
        remain = t_end - time.time()
        print(f"\r  Settling {wheel}... {remain:.0f}s  ", end='', flush=True)
        time.sleep(0.2)
    print()

    # Measure phase
    ser.reset_input_buffer()
    prev_pkt = None
    speeds = []
    t_end = time.time() + measure_s
    while time.time() < t_end:
        send_pwm(ser, l_pwm, r_pwm)
        time.sleep(0.02)
        for pkt in read_packets(ser):
            if prev_pkt is not None:
                if wheel == 'left':
                    delta = wrap16(pkt['l'] - prev_pkt['l'])
                else:
                    delta = -wrap16(pkt['r'] - prev_pkt['r'])
                spd = delta * MM_PER_TICK / 0.050
                speeds.append(spd)
            prev_pkt = pkt

        remain = t_end - time.time()
        if speeds:
            print(f"\r  Measuring {wheel}... {remain:.0f}s  "
                  f"{to_rpm(speeds[-1]):.0f} RPM  ", end='', flush=True)
    print()

    if not speeds:
        return 0, 0, []

    mean = sum(speeds) / len(speeds)
    std = (sum((v - mean)**2 for v in speeds) / len(speeds)) ** 0.5

    return to_rpm(mean), to_rpm(std), [to_rpm(s) for s in speeds]


def main():
    print("=" * 60)
    print(" ENCODER vs TACHOMETER COMPARISON")
    print(" Wheels must be elevated. ROS must be stopped.")
    print("=" * 60)
    print()

    ser = connect()
    stop(ser)

    rows = []

    try:
        while True:
            pwm_str = input("\nPWM to test (0-999, or 'done'): ").strip()
            if pwm_str.lower() == 'done':
                break
            try:
                pwm = int(pwm_str)
            except ValueError:
                print("  Enter a number or 'done'.")
                continue

            if pwm < 0 or pwm > 999:
                print("  PWM must be 0-999.")
                continue

            # --- LEFT MOTOR ---
            print(f"\n  === LEFT MOTOR at PWM {pwm} ===")
            input("  Press Enter when tachometer is ready on LEFT wheel...")
            l_rpm, l_std, _ = measure_single_wheel(ser, pwm, 'left',
                                                    SETTLE_TIME, MEASURE_TIME)
            print(f"  Encoder LEFT: {l_rpm:.1f} RPM (std {l_std:.1f})")
            stop(ser)
            print("  Left motor stopped.")
            tacho_l = input("  Tachometer LEFT RPM (or skip): ").strip()

            # --- RIGHT MOTOR ---
            print(f"\n  === RIGHT MOTOR at PWM {pwm} ===")
            input("  Press Enter when tachometer is ready on RIGHT wheel...")
            r_rpm, r_std, _ = measure_single_wheel(ser, pwm, 'right',
                                                    SETTLE_TIME, MEASURE_TIME)
            print(f"  Encoder RIGHT: {r_rpm:.1f} RPM (std {r_std:.1f})")
            stop(ser)
            print("  Right motor stopped.")
            tacho_r = input("  Tachometer RIGHT RPM (or skip): ").strip()

            row = {
                'pwm': pwm,
                'encoder_left_rpm': round(l_rpm, 1),
                'encoder_right_rpm': round(r_rpm, 1),
                'encoder_left_std': round(l_std, 1),
                'encoder_right_std': round(r_std, 1),
                'tacho_left_rpm': tacho_l if tacho_l else '',
                'tacho_right_rpm': tacho_r if tacho_r else '',
                'error_left_rpm': '',
                'error_left_pct': '',
                'error_right_rpm': '',
                'error_right_pct': '',
            }

            if tacho_l:
                try:
                    tl = float(tacho_l)
                    err_l = l_rpm - tl
                    pct_l = (err_l / tl * 100) if tl > 0 else 0
                    row['error_left_rpm'] = round(err_l, 1)
                    row['error_left_pct'] = round(pct_l, 1)
                    print(f"  Left error:  {err_l:+.1f} RPM ({pct_l:+.1f}%)")
                except ValueError:
                    pass

            if tacho_r:
                try:
                    tr = float(tacho_r)
                    err_r = r_rpm - tr
                    pct_r = (err_r / tr * 100) if tr > 0 else 0
                    row['error_right_rpm'] = round(err_r, 1)
                    row['error_right_pct'] = round(pct_r, 1)
                    print(f"  Right error: {err_r:+.1f} RPM ({pct_r:+.1f}%)")
                except ValueError:
                    pass

            rows.append(row)

    except KeyboardInterrupt:
        print("\nInterrupted.")
    finally:
        stop(ser)
        ser.close()
        print("Motor stopped. Serial closed.")

    if not rows:
        print("No data collected.")
        return

    # Save
    out_dir = os.path.expanduser('~/thesis_data/tachometer_comparison')
    os.makedirs(out_dir, exist_ok=True)
    ts = datetime.now().strftime('%Y%m%d_%H%M%S')
    csv_path = os.path.join(out_dir, f"encoder_vs_tacho_{ts}.csv")

    fields = ['pwm', 'encoder_left_rpm', 'encoder_right_rpm',
              'encoder_left_std', 'encoder_right_std',
              'tacho_left_rpm', 'tacho_right_rpm',
              'error_left_rpm', 'error_left_pct',
              'error_right_rpm', 'error_right_pct']
    U.save_csv(csv_path, fields, rows)

    # Summary
    print("\n" + "=" * 60)
    print(" SUMMARY")
    print("=" * 60)
    print(f" {'PWM':>5} | {'Enc L':>7} | {'Tacho L':>8} | {'Err L':>7} | "
          f"{'Enc R':>7} | {'Tacho R':>8} | {'Err R':>7}")
    print(" " + "-" * 62)
    for r in rows:
        tl = r.get('tacho_left_rpm', '')
        tr = r.get('tacho_right_rpm', '')
        el = r.get('error_left_pct', '')
        er = r.get('error_right_pct', '')
        el_s = f"{el:+.1f}%" if isinstance(el, (int, float)) else '-'
        er_s = f"{er:+.1f}%" if isinstance(er, (int, float)) else '-'
        print(f" {r['pwm']:5d} | {r['encoder_left_rpm']:6.1f}  | {str(tl):>8} | {el_s:>7} | "
              f"{r['encoder_right_rpm']:6.1f}  | {str(tr):>8} | {er_s:>7}")

    print(f"\nSaved: {csv_path}")


if __name__ == '__main__':
    main()
