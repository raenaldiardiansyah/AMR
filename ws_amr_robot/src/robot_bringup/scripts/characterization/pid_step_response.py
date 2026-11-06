#!/usr/bin/env python3
"""
pid_step_response.py
=============================================================
PURPOSE:
    Sends a speed command to the STM32, records actual wheel
    speeds from encoder telemetry in real time, then plots
    and saves the step response data so you can tune PID
    with real evidence instead of guessing.

HOW IT WORKS:
    1. Connects to STM32 serial port directly (no ROS needed)
    2. Sends a V:TARGET,W:0 command (both wheels same speed)
    3. Records encoder tick deltas at 20Hz for several seconds
    4. Converts ticks to actual speed in mm/s per wheel
    5. Plots: target speed vs actual speed for left and right
    6. Saves plot as PNG and raw data as CSV
    7. Prints analysis: rise time, overshoot, steady state error

WHAT TO LOOK FOR IN THE PLOT:
    - Rise time: how many seconds to reach 90% of target speed
      If too slow: increase Kp
      If too fast and overshoots: decrease Kp

    - Overshoot: does actual speed go above target before settling?
      If yes: decrease Kp, or add small Kd

    - Steady state error: does actual speed settle below target?
      If yes: increase Ki
      If Ki too high: speed oscillates around target

    - Symmetry: left and right wheel curves should overlap
      If they don't: wheels need different Kp or PWM_MIN

HOW TO RUN:
    # Kill ROS first
    ~/Autonomous-Mobile-Robot/ws_amr_robot/src/robot_bringup/scripts/tools/kill_robot.sh
    sleep 2

    # Run with default 300 mm/s target
    python3 ~/Autonomous-Mobile-Robot/ws_amr_robot/src/robot_bringup/scripts/characterization/pid_step_response.py

    # Run with custom target speed in mm/s
    python3 ~/Autonomous-Mobile-Robot/ws_amr_robot/src/robot_bringup/scripts/characterization/pid_step_response.py --speed 500

    # Run with custom duration
    python3 ~/Autonomous-Mobile-Robot/ws_amr_robot/src/robot_bringup/scripts/characterization/pid_step_response.py --speed 300 --duration 5

    # Test both forward and backward
    python3 ~/Autonomous-Mobile-Robot/ws_amr_robot/src/robot_bringup/scripts/characterization/pid_step_response.py --speed 300 --both-directions

IMPORTANT:
    - Robot wheels must be off the ground, OR robot must have
      clear space to drive straight for several meters.
    - Do NOT run ROS stack at the same time.
    - STM32 must have PID firmware flashed (not characterization).
=============================================================
"""

import serial
import json
import time
import csv
import os
import math
import argparse
from datetime import datetime

# ============================================================
# CONFIGURATION
# ============================================================
SERIAL_PORT  = '/dev/serial/by-id/usb-FTDI_FT232R_USB_UART_A5069RR4-if00-port0'
BAUD_RATE    = 115200

# Physical constants
WHEEL_DIAMETER  = 0.150
TICKS_PER_REV   = 2400.0   # 600 PPR x4 (robot kecil lama: 4557)
M_PER_TICK      = (math.pi * WHEEL_DIAMETER) / TICKS_PER_REV
MM_PER_TICK     = M_PER_TICK * 1000.0

# Test parameters (can be overridden by command line args)
# 150 mm/s is within the robot's real range (~210 mm/s max). 300 would
# saturate and show false steady-state error. Run 100/150/200 separately.
DEFAULT_SPEED_MMPS = 150   # mm/s
DEFAULT_DURATION   = 4.0   # seconds of recording after step

# Timing
SETTLE_BEFORE  = 0.5   # seconds at zero before step
SETTLE_AFTER   = 1.0   # seconds at zero after step (coast down)

OUTPUT_DIR  = os.path.expanduser('~/thesis_data/PID_tune_STM32')
os.makedirs(OUTPUT_DIR, exist_ok=True)
TIMESTAMP   = datetime.now().strftime('%Y%m%d_%H%M%S')


# ============================================================
# SERIAL HELPERS
# ============================================================

def connect_serial():
    print(f"Connecting to {SERIAL_PORT}...")
    try:
        ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=0.1)
        ser.reset_input_buffer()
        ser.reset_output_buffer()
        time.sleep(0.3)
        print("  Connected.")
        return ser
    except serial.SerialException as e:
        print(f"\nERROR: Cannot open serial port.")
        print(f"  {e}")
        print(f"\nMake sure ROS is killed first: ~/Autonomous-Mobile-Robot/ws_amr_robot/src/robot_bringup/scripts/tools/kill_robot.sh")
        raise


def send_velocity(ser, v_mmps, w_mradps=0):
    """Send V:v,W:w command to STM32."""
    cmd = f"V:{int(v_mmps)},W:{int(w_mradps)}\r\n"
    ser.write(cmd.encode('utf-8'))


def drain_and_get_latest(ser):
    """
    Drain the entire serial buffer and return the most recent
    valid JSON packet. Returns None if nothing valid found.
    """
    latest = None
    while ser.in_waiting > 0:
        try:
            line = ser.readline().decode('utf-8', errors='ignore').strip()
            if line.startswith('{') and line.endswith('}'):
                parsed = json.loads(line)
                latest = parsed
        except (json.JSONDecodeError, UnicodeDecodeError):
            pass
    return latest


# ============================================================
# STEP RESPONSE RECORDING
# ============================================================

def record_step_response(ser, target_speed_mmps, duration_seconds,
                          label="forward"):
    """
    Record a complete step response for both wheels.

    Returns a list of sample dicts:
        time_s, left_speed_mmps, right_speed_mmps,
        target_mmps, left_ticks_raw, right_ticks_raw
    """
    print(f"\n--- Recording: {label} at {target_speed_mmps} mm/s ---")

    samples = []

    # Phase 1: Settle at zero
    print(f"  Pre-step settle ({SETTLE_BEFORE}s at zero)...")
    t_settle = time.time()
    while (time.time() - t_settle) < SETTLE_BEFORE:
        send_velocity(ser, 0)
        time.sleep(0.05)

    ser.reset_input_buffer()

    # Get first packet to establish baseline ticks
    first_data = None
    deadline = time.time() + 1.0
    while time.time() < deadline:
        send_velocity(ser, 0)
        d = drain_and_get_latest(ser)
        if d is not None and 'l' in d and 'r' in d:
            first_data = d
            break
        time.sleep(0.05)

    if first_data is None:
        print("  ERROR: No telemetry from STM32. Check firmware and connection.")
        return []

    # Phase 2: Step response recording
    print(f"  Step applied. Recording {duration_seconds}s...")

    t_start      = time.time()
    prev_data    = first_data
    prev_time    = t_start
    step_applied = False

    while (time.time() - t_start) < duration_seconds:
        current_time = time.time()
        elapsed      = current_time - t_start

        # Apply step command on every iteration
        send_velocity(ser, target_speed_mmps)
        step_applied = True

        time.sleep(0.03)  # ~33Hz polling, faster than 20Hz telemetry

        data = drain_and_get_latest(ser)
        if data is None:
            continue
        if 'l' not in data or 'r' not in data:
            continue

        dt = current_time - prev_time
        if dt < 0.005:
            continue

        # Calculate actual speed from tick delta
        # Left wheel: positive ticks = forward (normal)
        # Right wheel: negative ticks = forward (polarity_right = -1)
        # We apply the same convention as odometry_node.py
        left_delta  =  (data['l'] - prev_data['l'])
        right_delta = -(data['r'] - prev_data['r'])  # flip sign, same as polarity_right=-1

        left_speed_mmps  = (left_delta  * MM_PER_TICK) / dt
        right_speed_mmps = (right_delta * MM_PER_TICK) / dt

        samples.append({
            'time_s':           round(elapsed, 4),
            'target_mmps':      target_speed_mmps,
            'left_speed_mmps':  round(left_speed_mmps,  2),
            'right_speed_mmps': round(right_speed_mmps, 2),
            'left_ticks_raw':   data['l'],
            'right_ticks_raw':  data['r'],
            'label':            label
        })

        prev_data = data
        prev_time = current_time

    # Phase 3: Coast down
    print(f"  Step removed. Coasting ({SETTLE_AFTER}s)...")
    t_coast = time.time()
    coast_start_time = time.time() - t_start  # elapsed time when coast starts

    while (time.time() - t_coast) < SETTLE_AFTER:
        send_velocity(ser, 0)

        data = drain_and_get_latest(ser)
        current_time = time.time()

        if data is not None and 'l' in data and 'r' in data:
            dt = current_time - prev_time
            if dt >= 0.005:
                elapsed = (current_time - t_start)

                left_delta  =  (data['l'] - prev_data['l'])
                right_delta = -(data['r'] - prev_data['r'])

                left_speed_mmps  = (left_delta  * MM_PER_TICK) / dt
                right_speed_mmps = (right_delta * MM_PER_TICK) / dt

                samples.append({
                    'time_s':           round(elapsed, 4),
                    'target_mmps':      0,  # command is now zero
                    'left_speed_mmps':  round(left_speed_mmps,  2),
                    'right_speed_mmps': round(right_speed_mmps, 2),
                    'left_ticks_raw':   data['l'],
                    'right_ticks_raw':  data['r'],
                    'label':            label + '_coast'
                })

                prev_data = data
                prev_time = current_time

        time.sleep(0.03)

    print(f"  Done. {len(samples)} samples collected.")
    return samples


# ============================================================
# ANALYSIS
# ============================================================

def analyze_step_response(samples, target_mmps, wheel='left'):
    """
    Analyze step response for one wheel.
    Returns dict with rise_time, overshoot, steady_state_error.
    """
    key = f'{wheel}_speed_mmps'

    # Only look at the active step period (target > 0)
    active = [s for s in samples if s['target_mmps'] == target_mmps]

    if len(active) < 5:
        return None

    speeds = [s[key] for s in active]
    times  = [s['time_s'] for s in active]

    # Smooth with a simple moving average to reduce noise
    window = 5
    smoothed = []
    for i in range(len(speeds)):
        start = max(0, i - window // 2)
        end   = min(len(speeds), i + window // 2 + 1)
        smoothed.append(sum(speeds[start:end]) / (end - start))

    # Rise time: time from step to first reaching 90% of target
    threshold_90 = abs(target_mmps) * 0.9
    rise_time = None
    for i, (t, s) in enumerate(zip(times, smoothed)):
        if abs(s) >= threshold_90:
            rise_time = t
            break

    # Overshoot: max speed above target during active period
    peak_speed  = max(smoothed) if target_mmps > 0 else min(smoothed)
    overshoot   = ((abs(peak_speed) - abs(target_mmps)) / abs(target_mmps)) * 100
    overshoot   = max(0, overshoot)  # negative overshoot = undershoot, show as 0

    # Steady state: average of last 30% of active samples
    ss_start = int(len(smoothed) * 0.7)
    ss_speed = sum(smoothed[ss_start:]) / max(1, len(smoothed[ss_start:]))
    ss_error = abs(target_mmps) - abs(ss_speed)
    ss_error_pct = (ss_error / abs(target_mmps)) * 100

    return {
        'wheel':          wheel,
        'target_mmps':    target_mmps,
        'rise_time_s':    round(rise_time, 3) if rise_time else None,
        'overshoot_pct':  round(overshoot, 1),
        'ss_error_mmps':  round(ss_error, 1),
        'ss_error_pct':   round(ss_error_pct, 1),
        'peak_speed':     round(peak_speed, 1),
        'ss_speed':       round(ss_speed, 1)
    }


def print_analysis(analysis_left, analysis_right, target_mmps):
    print("\n" + "=" * 60)
    print(f"STEP RESPONSE ANALYSIS  (target = {target_mmps} mm/s)")
    print("=" * 60)

    for a in [analysis_left, analysis_right]:
        if a is None:
            continue
        print(f"\n  [{a['wheel'].upper()} WHEEL]")

        if a['rise_time_s'] is not None:
            print(f"  Rise time (0 to 90%):  {a['rise_time_s']:.3f} s")
        else:
            print(f"  Rise time:             did not reach 90% of target")

        print(f"  Peak speed:            {a['peak_speed']:.1f} mm/s")
        print(f"  Overshoot:             {a['overshoot_pct']:.1f}%")
        print(f"  Steady state speed:    {a['ss_speed']:.1f} mm/s")
        print(f"  Steady state error:    {a['ss_error_mmps']:.1f} mm/s  "
              f"({a['ss_error_pct']:.1f}%)")

        # Diagnosis
        print(f"  Diagnosis:")
        if a['rise_time_s'] is None:
            print(f"    --> Motor never reached target. Kp too low or PWM_MIN too low.")
        elif a['rise_time_s'] > 1.5:
            print(f"    --> Rise time slow. Consider increasing Kp.")
        elif a['rise_time_s'] < 0.2:
            print(f"    --> Rise time very fast. Watch for overshoot.")

        if a['overshoot_pct'] > 20:
            print(f"    --> Overshoot too high ({a['overshoot_pct']:.0f}%). "
                  f"Decrease Kp or add small Kd.")
        elif a['overshoot_pct'] > 5:
            print(f"    --> Slight overshoot ({a['overshoot_pct']:.0f}%). Acceptable.")
        else:
            print(f"    --> No significant overshoot. Good.")

        if a['ss_error_pct'] > 15:
            print(f"    --> Large steady state error ({a['ss_error_pct']:.0f}%). "
                  f"Increase Ki.")
        elif a['ss_error_pct'] > 5:
            print(f"    --> Small steady state error ({a['ss_error_pct']:.0f}%). "
                  f"Ki may need slight increase.")
        else:
            print(f"    --> Steady state error acceptable. Ki is working.")

    # Symmetry check
    if analysis_left and analysis_right:
        print(f"\n  [SYMMETRY CHECK]")
        ss_diff = abs(analysis_left['ss_speed'] - analysis_right['ss_speed'])
        ss_diff_pct = (ss_diff / abs(target_mmps)) * 100
        print(f"  Left ss speed:   {analysis_left['ss_speed']:.1f} mm/s")
        print(f"  Right ss speed:  {analysis_right['ss_speed']:.1f} mm/s")
        print(f"  Difference:      {ss_diff:.1f} mm/s  ({ss_diff_pct:.1f}%)")

        if ss_diff_pct > 10:
            print(f"  --> Significant asymmetry. Robot will drift.")
            print(f"  --> Consider separate Kp or Ki per wheel.")
        else:
            print(f"  --> Good symmetry. Robot should go straight.")

    print()


def print_tuning_suggestion(analysis_left, analysis_right, target_mmps):
    """Print concrete next Kp/Ki values to try based on analysis."""
    print("=" * 60)
    print("SUGGESTED NEXT PID VALUES TO TRY")
    print("=" * 60)
    print("Based on the analysis above, here are starting suggestions.")
    print("Apply ONE change at a time and re-run this script to verify.")
    print()

    for a in [analysis_left, analysis_right]:
        if a is None:
            continue
        wheel = a['wheel']
        print(f"  [{wheel.upper()} WHEEL]")

        if a['rise_time_s'] is None or a['rise_time_s'] > 1.5:
            print(f"  --> Kp: increase by 0.5")
        elif a['overshoot_pct'] > 20:
            print(f"  --> Kp: decrease by 0.5")
        else:
            print(f"  --> Kp: keep current value")

        if a['ss_error_pct'] > 15:
            print(f"  --> Ki: increase by 0.5")
        elif a['ss_error_pct'] > 5:
            print(f"  --> Ki: increase by 0.2")
        else:
            print(f"  --> Ki: keep current value")

        print()


# ============================================================
# SAVE DATA
# ============================================================

def save_csv(all_samples, filename):
    if not all_samples:
        return
    fieldnames = ['time_s', 'target_mmps', 'left_speed_mmps',
                  'right_speed_mmps', 'left_ticks_raw',
                  'right_ticks_raw', 'label']
    with open(filename, 'w', newline='') as f:
        writer = csv.DictWriter(f, fieldnames=fieldnames)
        writer.writeheader()
        writer.writerows(all_samples)
    print(f"Data saved to: {filename}")


def plot_response(all_samples, target_mmps, png_filename):
    """
    Plot the step response. Uses matplotlib if available.
    If matplotlib is not installed, prints instructions.
    """
    try:
        import matplotlib
        matplotlib.use('Agg')  # Non-interactive backend, works without display
        import matplotlib.pyplot as plt
    except ImportError:
        print("\nmatplotlib not installed. Cannot generate plot.")
        print("Install with: pip3 install matplotlib --break-system-packages")
        print("Then re-run this script.")
        return

    if not all_samples:
        print("No samples to plot.")
        return

    fig, axes = plt.subplots(2, 1, figsize=(12, 8), sharex=True)
    fig.suptitle(f'PID Step Response  |  Target: {target_mmps} mm/s',
                 fontsize=14, fontweight='bold')

    colors = {
        'left_forward':   '#2196F3',
        'left_backward':  '#1565C0',
        'right_forward':  '#F44336',
        'right_backward': '#B71C1C',
        'target':         '#4CAF50',
    }

    # Group samples by label
    labels_seen = set(s['label'] for s in all_samples)

    for ax_idx, wheel in enumerate(['left', 'right']):
        ax = axes[ax_idx]
        key = f'{wheel}_speed_mmps'

        for label in sorted(labels_seen):
            if 'coast' in label:
                continue
            group = [s for s in all_samples if s['label'] == label]
            if not group:
                continue

            times  = [s['time_s'] for s in group]
            speeds = [s[key] for s in group]
            target = [s['target_mmps'] for s in group]

            direction = 'forward' if target_mmps > 0 else 'backward'
            color_key = f'{wheel}_{direction}'
            color = colors.get(color_key, '#9E9E9E')

            ax.plot(times, speeds, color=color, linewidth=1.5,
                    alpha=0.7, label=f'Actual ({label})')
            ax.plot(times, target, color=colors['target'],
                    linewidth=2, linestyle='--', alpha=0.8,
                    label='Target')

        ax.axhline(y=0, color='black', linewidth=0.5, alpha=0.3)
        ax.set_ylabel('Speed (mm/s)', fontsize=11)
        ax.set_title(f'{wheel.upper()} Wheel', fontsize=12)
        ax.legend(loc='upper right', fontsize=9)
        ax.grid(True, alpha=0.3)

        # Add tolerance band (±10% of target)
        tol = abs(target_mmps) * 0.1
        ax.axhspan(target_mmps - tol, target_mmps + tol,
                   alpha=0.1, color='green', label='±10% tolerance')

    axes[-1].set_xlabel('Time (seconds)', fontsize=11)
    plt.tight_layout()
    plt.savefig(png_filename, dpi=150, bbox_inches='tight')
    print(f"Plot saved to: {png_filename}")


# ============================================================
# MAIN
# ============================================================

def main():
    parser = argparse.ArgumentParser(
        description='PID Step Response Logger for AMR Robot')
    parser.add_argument('--speed', type=int, default=DEFAULT_SPEED_MMPS,
                        help=f'Target speed in mm/s (default: {DEFAULT_SPEED_MMPS})')
    parser.add_argument('--duration', type=float, default=DEFAULT_DURATION,
                        help=f'Recording duration in seconds (default: {DEFAULT_DURATION})')
    parser.add_argument('--both-directions', action='store_true',
                        help='Test both forward and backward')
    args = parser.parse_args()

    target = args.speed
    duration = args.duration

    print("=" * 60)
    print("PID STEP RESPONSE LOGGER")
    print(f"Target speed  : {target} mm/s  ({target/1000:.2f} m/s)")
    print(f"Duration      : {duration} s")
    print(f"Both dirs     : {args.both_directions}")
    print()
    print("SAFETY: Make sure wheels are off ground or robot has")
    print("        clear space. Kill ROS before running.")
    print()

    input("Press ENTER to start, or Ctrl+C to cancel...")

    ser = connect_serial()
    all_samples = []

    try:
        # Forward step
        samples_fwd = record_step_response(
            ser, target, duration, label='forward')
        all_samples.extend(samples_fwd)

        # Backward step if requested
        if args.both_directions:
            time.sleep(1.0)
            samples_bwd = record_step_response(
                ser, -target, duration, label='backward')
            all_samples.extend(samples_bwd)

    except KeyboardInterrupt:
        print("\nInterrupted by user.")
    finally:
        # Always stop motors
        for _ in range(10):
            send_velocity(ser, 0)
            time.sleep(0.05)
        ser.close()
        print("Serial port closed. Motors stopped.")

    if not all_samples:
        print("No data collected.")
        return

    # Save CSV
    csv_file = os.path.join(OUTPUT_DIR, f'WITH_PID_step_{int(target)}mmps_{TIMESTAMP}.csv')
    save_csv(all_samples, csv_file)

    # Analyze forward response
    fwd_samples = [s for s in all_samples
                   if s['label'] == 'forward']
    if fwd_samples:
        analysis_left  = analyze_step_response(fwd_samples, target, 'left')
        analysis_right = analyze_step_response(fwd_samples, target, 'right')
        print_analysis(analysis_left, analysis_right, target)
        print_tuning_suggestion(analysis_left, analysis_right, target)

    # Generate plot
    png_file = os.path.join(OUTPUT_DIR, f'WITH_PID_step_{int(target)}mmps_{TIMESTAMP}.png')
    plot_response(all_samples, target, png_file)

    print("\nDone. Send me:")
    print(f"  1. The plot PNG:  {png_file}")
    print(f"  2. The CSV file:  {csv_file}")
    print(f"  3. Your current Kp and Ki values in motor_control.c")
    print("And I will calculate the next values to try.")


if __name__ == '__main__':
    main()
