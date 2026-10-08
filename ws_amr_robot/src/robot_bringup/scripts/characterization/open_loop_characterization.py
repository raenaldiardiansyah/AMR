#!/usr/bin/env python3
"""
open_loop_characterization.py
=============================================================
PURPOSE:
    This script runs on the Raspberry Pi and talks to the STM32
    running freertos_characterization.c firmware.

    It sweeps PWM values from 0 to 999 in steps of 50, measures
    how fast each wheel spins, and saves the result to a CSV file.
    You then use that CSV to find the deadband and calculate Kp.

WHAT IT DOES STEP BY STEP:
    1. Connect to STM32 via serial port.
    2. For each PWM value in [0, 50, 100, 150, ..., 950, 999]:
       a. Send "P:PWM,0\r\n" to run only the LEFT wheel.
       b. Wait 2 seconds for the motor to reach steady speed.
       c. Record encoder ticks for 3 seconds.
       d. Calculate average speed in m/s from tick rate.
       e. Stop the motor for 1 second before next step.
    3. Repeat step 2 but for RIGHT wheel only ("P:0,PWM\r\n").
    4. Save all data to a CSV file with timestamp in filename.
    5. Print a summary table to the terminal.

HOW TO RUN:
    python3 open_loop_characterization.py

    The robot must be elevated (wheels off ground) or in a space
    where it can drive forward freely for at least 2 meters.
    Recommended: lift robot off ground and support the chassis.

SAFETY WARNING:
    The STM32 firmware has a 2-second timeout. If this script
    crashes mid-run, the motors will stop automatically after 2s.
    Do NOT run this while the ROS stack is also running,
    because both will try to use the same serial port.

OUTPUT:
    CSV file named: characterization_YYYYMMDD_HHMMSS.csv
    Columns: wheel, pwm, avg_speed_mps, std_speed_mps,
             total_ticks, duration_seconds, ticks_per_second
=============================================================
"""

import serial
import json
import time
import csv
import os
import math
from datetime import datetime


# ============================================================
# CONFIGURATION - Change these if your setup is different
# ============================================================

SERIAL_PORT = '/dev/serial/by-id/usb-FTDI_FT232R_USB_UART_A5069RR4-if00-port0'
BAUD_RATE   = 115200

# Physical robot constants (same as your odometry_node.py)
WHEEL_DIAMETER   = 0.150   # meters
TICKS_PER_REV    = 2400.0  # encoder ticks per full wheel revolution
M_PER_TICK       = (math.pi * WHEEL_DIAMETER) / TICKS_PER_REV  # meters per tick

# PWM sweep settings
PWM_START   = 0
PWM_END     = 999
PWM_STEP    = 50    # We test 0, 50, 100, 150, ..., 950, 999
# 999 is added explicitly at the end to always test maximum

SETTLE_TIME    = 2.0   # seconds: wait for motor to reach steady speed
MEASURE_TIME   = 3.0   # seconds: collect data during this window
COOLDOWN_TIME  = 1.0   # seconds: stop between tests, protect motor

# Output file location. Name makes it obvious this is the RAW (no-PID) data.
OUTPUT_DIR  = os.path.expanduser('~/thesis_data/PID_tune_STM32')
os.makedirs(OUTPUT_DIR, exist_ok=True)
TIMESTAMP   = datetime.now().strftime('%Y%m%d_%H%M%S')
OUTPUT_FILE = os.path.join(OUTPUT_DIR, f'RAW_openloop_no_pid_{TIMESTAMP}.csv')


# ============================================================
# HELPER FUNCTIONS
# ============================================================

def connect_serial():
    """Open the serial port and return the serial object."""
    print(f"\nConnecting to {SERIAL_PORT} at {BAUD_RATE} baud...")
    try:
        ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=0.5)
        ser.reset_input_buffer()
        ser.reset_output_buffer()
        time.sleep(0.5)  # Give STM32 a moment to be ready
        print("  Connected.")
        return ser
    except serial.SerialException as e:
        print(f"\nERROR: Could not open serial port.")
        print(f"  Reason: {e}")
        print(f"\nPossible causes:")
        print(f"  1. STM32 is not plugged in.")
        print(f"  2. Wrong serial port path.")
        print(f"  3. Another program (ROS bridge) is using the port.")
        print(f"     Kill all ROS nodes first: ~/Autonomous-Mobile-Robot/ws_amr_robot/src/robot_bringup/scripts/tools/kill_robot.sh")
        raise


def send_pwm(ser, left_pwm, right_pwm):
    """
    Send a PWM command to the STM32.
    Format: "P:LEFT_PWM,RIGHT_PWM\r\n"
    """
    cmd = f"P:{int(left_pwm)},{int(right_pwm)}\r\n"
    ser.write(cmd.encode('utf-8'))


def read_latest_telemetry(ser):
    """
    Read all pending bytes from serial buffer and return the
    most recent complete JSON line. Returns None if nothing valid.

    We drain the entire buffer and keep only the last valid line.
    This is the same approach as stm32_bridge.py:
    we do not want old stale data, we want the freshest reading.
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


def read_all_telemetry(ser):
    """Return a list of ALL complete JSON packets waiting in the buffer, in
    order. Unlike read_latest_telemetry this keeps every packet, so no ticks
    are lost between consecutive 50ms firmware messages. That keeps each
    tick delta small and the 16-bit wrap correction safe."""
    packets = []
    while ser.in_waiting > 0:
        try:
            line = ser.readline().decode('utf-8', errors='ignore').strip()
            if line.startswith('{') and line.endswith('}'):
                packets.append(json.loads(line))
        except (json.JSONDecodeError, UnicodeDecodeError):
            pass
    return packets


def measure_wheel_speed(ser, left_pwm, right_pwm):
    """
    Command a specific PWM, wait for the motor to settle,
    then measure the tick rate over MEASURE_TIME seconds.

    Returns a dict with:
        avg_speed_mps   : average wheel speed in meters per second
        std_speed_mps   : standard deviation of speed samples
        total_ticks     : total ticks counted during measurement
        duration        : actual measured duration in seconds
        ticks_per_second: average tick rate
        samples         : number of telemetry packets received

    The wheel we are measuring depends on which PWM is non-zero.
    If left_pwm > 0, we measure left wheel. Otherwise right wheel.
    """
    measuring_left = (left_pwm > 0)

    # Step 1: Send command and drain any old serial data
    send_pwm(ser, left_pwm, right_pwm)
    ser.reset_input_buffer()
    time.sleep(0.05)

    # Step 2: Wait for motor to reach steady-state speed.
    # During this time we keep sending the command every 500ms
    # to prevent the 2-second safety timeout from triggering.
    print(f"    Settling ({SETTLE_TIME:.0f}s)...", end='', flush=True)
    settle_start = time.time()
    while (time.time() - settle_start) < SETTLE_TIME:
        send_pwm(ser, left_pwm, right_pwm)  # Refresh timeout
        time.sleep(0.5)
    print(" done.")

    # Drain buffer again so measurement starts fresh
    ser.reset_input_buffer()

    # Step 3: Collect tick data during measurement window.
    # We record the encoder ticks at the start and end.
    # STM32 sends accumulated total ticks since power-on.
    # Speed = (ticks_end - ticks_start) / duration * m_per_tick

    # Read one packet to get the starting tick value
    print(f"    Measuring ({MEASURE_TIME:.0f}s)...", end='', flush=True)

    first_data = None
    first_time = None

    # Wait up to 1 second for the first valid packet
    deadline = time.time() + 1.0
    while time.time() < deadline:
        send_pwm(ser, left_pwm, right_pwm)
        data = read_latest_telemetry(ser)
        if data is not None and 'l' in data and 'r' in data:
            first_data = data
            first_time = time.time()
            break
        time.sleep(0.05)

    if first_data is None:
        print(" ERROR: No telemetry received from STM32.")
        print("   Check that STM32 is running the characterization firmware.")
        return None

    # Process EVERY telemetry packet (not just the newest). The firmware tick
    # field is a 16-bit counter that wraps at 65536. Between two consecutive
    # 50ms packets the counter can only move a few thousand ticks, far below
    # the 32767 wrap threshold, so each per-packet delta is always safe to
    # wrap-correct. We sum these into accum_ticks. Discarding packets (the old
    # bug) let the gap exceed 32767 and corrupted the count.
    key = 'l' if measuring_left else 'r'
    prev_tick   = first_data[key]
    accum_ticks = 0
    speed_samples = []
    last_time   = first_time
    NOMINAL_DT  = 0.05   # firmware telemetry period, for the jitter estimate

    def wrap16(d):
        d %= 65536
        if d >= 32768:
            d -= 65536
        return d

    measure_end = time.time() + MEASURE_TIME

    while time.time() < measure_end:
        send_pwm(ser, left_pwm, right_pwm)  # Keep refreshing the timeout
        time.sleep(0.02)

        packets = read_all_telemetry(ser)
        for pkt in packets:
            if key not in pkt:
                continue
            delta = wrap16(pkt[key] - prev_tick)
            accum_ticks += delta
            prev_tick = pkt[key]
            # per-packet speed, jitter indicator only (nominal 50ms period)
            speed_samples.append(abs(delta) * M_PER_TICK / NOMINAL_DT)
        if packets:
            last_time = time.time()

    print(" done.")

    if len(speed_samples) == 0:
        print("    WARNING: No valid speed samples collected.")
        print("    Motor may be below deadband (not spinning at all).")
        return {
            'avg_speed_mps':    0.0,
            'std_speed_mps':    0.0,
            'total_ticks':      0,
            'duration':         MEASURE_TIME,
            'ticks_per_second': 0.0,
            'samples':          0
        }

    # --- WINDOW-BASED SPEED (robust, wrap-corrected) ---
    # Use the wrap-corrected TOTAL ticks over the WHOLE window divided by the
    # total elapsed time. This is immune to both the 16-bit counter wrap and
    # the per-read dt jitter, giving a clean steady speed.
    total_ticks     = accum_ticks
    actual_duration = last_time - first_time

    if actual_duration <= 0.001:
        return None

    avg_tps   = abs(total_ticks) / actual_duration
    avg_speed = avg_tps * M_PER_TICK

    # std of the per-sample speeds is kept only as a JITTER indicator (it is
    # dominated by dt noise, not real motor variation, so do not read it as
    # motor instability).
    if len(speed_samples) > 1:
        mean_s   = sum(speed_samples) / len(speed_samples)
        variance = sum((s - mean_s) ** 2 for s in speed_samples) / len(speed_samples)
        std_speed = math.sqrt(variance)
    else:
        std_speed = 0.0

    return {
        'avg_speed_mps':    avg_speed,
        'std_speed_mps':    std_speed,
        'total_ticks':      total_ticks,
        'duration':         actual_duration,
        'ticks_per_second': avg_tps,
        'samples':          len(speed_samples)
    }


def stop_motors(ser):
    """Send stop command multiple times to be sure."""
    for _ in range(5):
        send_pwm(ser, 0, 0)
        time.sleep(0.1)


def run_sweep(ser, wheel_name, pwm_values):
    """
    Run a full PWM sweep for one wheel.

    wheel_name: 'left' or 'right'
    pwm_values: list of PWM integers to test

    Returns a list of result dicts.
    """
    results = []

    for pwm in pwm_values:
        print(f"\n  [{wheel_name.upper()} WHEEL] PWM = {pwm}")

        if wheel_name == 'left':
            result = measure_wheel_speed(ser, left_pwm=pwm, right_pwm=0)
        else:
            result = measure_wheel_speed(ser, left_pwm=0, right_pwm=pwm)

        if result is not None:
            speed = result['avg_speed_mps']
            std   = result['std_speed_mps']
            tps   = result['ticks_per_second']
            n     = result['samples']
            print(f"    Result: {speed:.4f} m/s  (std={std:.4f}, tps={tps:.1f}, n={n})")

            results.append({
                'wheel':           wheel_name,
                'pwm':             pwm,
                'avg_speed_mps':   round(speed, 6),
                'std_speed_mps':   round(std,   6),
                'total_ticks':     result['total_ticks'],
                'duration_seconds':round(result['duration'], 3),
                'ticks_per_second':round(tps, 2),
                'samples':         n
            })
        else:
            # If measurement failed, record zeros so CSV stays complete
            print(f"    Result: FAILED - recording zeros")
            results.append({
                'wheel':           wheel_name,
                'pwm':             pwm,
                'avg_speed_mps':   0.0,
                'std_speed_mps':   0.0,
                'total_ticks':     0,
                'duration_seconds':MEASURE_TIME,
                'ticks_per_second':0.0,
                'samples':         0
            })

        # Stop motor between measurements and wait for wheel to fully stop
        print(f"    Stopping for {COOLDOWN_TIME:.0f}s...")
        stop_motors(ser)
        time.sleep(COOLDOWN_TIME)

    return results


def save_csv(all_results):
    """Write all results to a CSV file."""
    fieldnames = [
        'wheel', 'pwm', 'avg_speed_mps', 'std_speed_mps',
        'total_ticks', 'duration_seconds', 'ticks_per_second', 'samples'
    ]

    with open(OUTPUT_FILE, 'w', newline='') as f:
        writer = csv.DictWriter(f, fieldnames=fieldnames)
        writer.writeheader()
        writer.writerows(all_results)

    print(f"\nData saved to: {OUTPUT_FILE}")


def print_summary(all_results):
    """Print a readable summary table to the terminal."""
    print("\n" + "=" * 70)
    print("CHARACTERIZATION SUMMARY")
    print("=" * 70)
    print(f"{'Wheel':<8} {'PWM':<6} {'Speed (m/s)':<14} {'Std Dev':<12} {'Ticks/s':<12} {'Samples'}")
    print("-" * 70)

    for r in all_results:
        moving = "*" if r['avg_speed_mps'] > 0.001 else " "
        print(f"{r['wheel']:<8} {r['pwm']:<6} "
              f"{r['avg_speed_mps']:<14.4f} "
              f"{r['std_speed_mps']:<12.4f} "
              f"{r['ticks_per_second']:<12.1f} "
              f"{r['samples']}{moving}")

    print("-" * 70)
    print("  * = wheel is spinning (above noise floor)")
    print()

    # Find deadband (last PWM where speed is still ~0)
    for wheel_name in ['left', 'right']:
        wheel_data = [r for r in all_results if r['wheel'] == wheel_name]
        deadband_pwm = 0
        for r in wheel_data:
            if r['avg_speed_mps'] < 0.005:  # Less than 5mm/s = not moving
                deadband_pwm = r['pwm']
            else:
                break  # First PWM where it moves is just above deadband

        # Find linear region: look at PWM values where speed > 0
        moving_data = [r for r in wheel_data if r['avg_speed_mps'] > 0.005]

        if len(moving_data) >= 2:
            # Simple linear fit: use first and last point of linear region
            # A proper linear regression would be better, but this gives
            # a good enough starting estimate for Kp calculation.
            # We fit: speed = gain * (pwm - deadband)
            # gain = delta_speed / delta_pwm
            first = moving_data[0]
            last  = moving_data[-1]
            delta_pwm   = last['pwm'] - first['pwm']
            delta_speed = last['avg_speed_mps'] - first['avg_speed_mps']

            if delta_pwm > 0:
                gain = delta_speed / delta_pwm  # m/s per PWM count

                # Kp calculation:
                # In your PID, error is in mm/s (target - actual).
                # Output is PWM (0 to 999).
                # Kp should convert mm/s error to PWM output.
                # If gain = speed_change / pwm_change (in m/s per PWM),
                # then 1/gain = pwm_change per m/s change.
                # Convert to mm/s: Kp_start = (1/gain) / 1000
                # This is the PWM per mm/s of error, which is Kp.
                kp_start = (1.0 / gain) / 1000.0

                print(f"[{wheel_name.upper()} WHEEL ANALYSIS]")
                print(f"  Deadband:         PWM <= {deadband_pwm}")
                print(f"  First movement:   PWM ~  {moving_data[0]['pwm']}")
                print(f"  System gain:      {gain:.6f} m/s per PWM count")
                print(f"  Suggested Kp:     {kp_start:.4f}")
                print(f"  (This Kp gives roughly 1:1 correction at the")
                print(f"   first step. Tune up from here if too slow.)")
                print()
        else:
            print(f"[{wheel_name.upper()} WHEEL] Not enough moving data to compute gain.")
            print(f"  Deadband may be higher than PWM={PWM_END}")
            print()


# ============================================================
# MAIN ENTRY POINT
# ============================================================

def main():
    print("=" * 60)
    print("OPEN LOOP MOTOR CHARACTERIZATION")
    print(f"M_PER_TICK = {M_PER_TICK:.8f} m/tick")
    print(f"PWM range  = {PWM_START} to {PWM_END}, step {PWM_STEP}")
    print(f"Settle time  = {SETTLE_TIME}s per step")
    print(f"Measure time = {MEASURE_TIME}s per step")
    print(f"Cooldown     = {COOLDOWN_TIME}s between steps")
    print()

    # Build PWM list
    pwm_list = list(range(PWM_START, PWM_END, PWM_STEP))
    if PWM_END not in pwm_list:
        pwm_list.append(PWM_END)

    total_steps = len(pwm_list) * 2  # Two wheels
    time_per_step = SETTLE_TIME + MEASURE_TIME + COOLDOWN_TIME
    total_minutes = (total_steps * time_per_step) / 60.0

    print(f"Total steps: {total_steps} ({len(pwm_list)} per wheel)")
    print(f"Estimated time: {total_minutes:.1f} minutes")
    print()
    print("SAFETY REMINDERS:")
    print("  1. Make sure NO ROS nodes are running (kill_robot.sh)")
    print("  2. Robot wheels should be off the ground, OR")
    print("     robot should have clear space to drive forward.")
    print("  3. Keep hands away from wheels during the test.")
    print()

    input("Press ENTER to start, or Ctrl+C to cancel...")

    # Connect to STM32
    ser = connect_serial()

    all_results = []

    try:
        # Wait for the STM32 startup message
        print("\nWaiting for STM32 ready signal...")
        time.sleep(1.0)
        ser.reset_input_buffer()

        # ---- SWEEP LEFT WHEEL ----
        print("\n" + "=" * 60)
        print("PHASE 1: LEFT WHEEL SWEEP")
        print("Right wheel PWM = 0 (stopped)")
        print("=" * 60)
        left_results = run_sweep(ser, 'left', pwm_list)
        all_results.extend(left_results)

        # ---- REST BETWEEN WHEELS ----
        print("\nLeft wheel sweep done. Resting for 3 seconds...")
        stop_motors(ser)
        time.sleep(3.0)

        # ---- SWEEP RIGHT WHEEL ----
        print("\n" + "=" * 60)
        print("PHASE 2: RIGHT WHEEL SWEEP")
        print("Left wheel PWM = 0 (stopped)")
        print("=" * 60)
        right_results = run_sweep(ser, 'right', pwm_list)
        all_results.extend(right_results)

    except KeyboardInterrupt:
        print("\n\nTest interrupted by user.")
        print("Stopping motors...")
    finally:
        # Always stop motors on exit, no matter what happened
        stop_motors(ser)
        ser.close()
        print("Serial port closed.")

    # Save and display results
    if len(all_results) > 0:
        save_csv(all_results)
        print_summary(all_results)
    else:
        print("No data collected. Nothing saved.")


if __name__ == '__main__':
    main()
