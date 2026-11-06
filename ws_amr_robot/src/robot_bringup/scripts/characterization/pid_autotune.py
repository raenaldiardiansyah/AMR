#!/usr/bin/env python3
"""
pid_autotune.py
=============================================================
PURPOSE:
    Automatically calibrates PID gains for both robot wheels
    using the Ziegler-Nichols Ultimate Gain method.

    This is the same principle as Klipper's PID_CALIBRATE
    command for printer nozzles, adapted for DC motor speed
    control.

HOW IT WORKS (step by step):
    1. Start with a low Kp (Ki=0, Kd=0 disabled during search)
    2. Send a target speed command to the wheel
    3. Measure actual speed from encoder feedback
    4. Gradually increase Kp until the speed oscillates
       continuously (not dying out, not growing forever)
    5. Record that critical Kp value as Ku
    6. Measure the period of one full oscillation as Pu
    7. Calculate final Kp, Ki, Kd from Ziegler-Nichols formula:
         Kp = 0.6 * Ku
         Ki = 2 * Kp / Pu
         Kd = Kp * Pu / 8
    8. Repeat for the other wheel
    9. Print final values and save to a config file

IMPORTANT NOTES:
    - Robot wheels MUST be off the ground during this test
    - Kill ROS completely before running: ~/Autonomous-Mobile-Robot/ws_amr_robot/src/robot_bringup/scripts/tools/kill_robot.sh
    - This test takes about 5 to 10 minutes total
    - The wheels will oscillate intentionally during the test
      This is normal and expected behavior

HOW TO RUN:
    ~/Autonomous-Mobile-Robot/ws_amr_robot/src/robot_bringup/scripts/tools/kill_robot.sh && sleep 3
    python3 ~/Autonomous-Mobile-Robot/ws_amr_robot/src/robot_bringup/scripts/characterization/pid_autotune.py

    Options:
    --target-speed  Target speed in mm/s (default: 300)
    --left-only     Only tune left wheel
    --right-only    Only tune right wheel
    --kp-start      Starting Kp value (default: 0.5)
    --kp-step       Kp increment per cycle (default: 0.5)
    --kp-max        Maximum Kp to try (default: 20.0)
    --cycles        Oscillation cycles to measure (default: 5)

WHAT TO DO WITH THE RESULTS:
    The script prints exact values to put in motor_control.c:
        g_pid_left.kp  = X.Xf;
        g_pid_left.ki  = X.Xf;
        g_pid_left.kd  = X.Xf;
    Copy and paste these into Motor_Init() in motor_control.c,
    then rebuild and flash.
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
from collections import deque

# ============================================================
# CONFIGURATION
# ============================================================
SERIAL_PORT = '/dev/serial/by-id/usb-FTDI_FT232R_USB_UART_A5069RR4-if00-port0'
BAUD_RATE   = 115200

WHEEL_DIAMETER = 0.150
TICKS_PER_REV  = 2400.0
M_PER_TICK     = (math.pi * WHEEL_DIAMETER) / TICKS_PER_REV
MM_PER_TICK    = M_PER_TICK * 1000.0

# Autotune parameters
DEFAULT_TARGET_SPEED = 300   # mm/s - moderate speed for safe tuning
DEFAULT_KP_START     = 0.5   # start searching from this Kp
DEFAULT_KP_STEP      = 0.5   # increase Kp by this each cycle
DEFAULT_KP_MAX       = 20.0  # stop if Kp reaches this (safety)
DEFAULT_CYCLES       = 5     # measure this many oscillation cycles
SAMPLE_RATE_HZ       = 25    # how fast we sample (must be > 2x oscillation freq)
SAMPLE_DT            = 1.0 / SAMPLE_RATE_HZ

# Oscillation detection parameters
# Speed must cross the target line this many times to count as oscillation
MIN_CROSSINGS_FOR_OSCILLATION = 6
# Oscillation amplitude must be at least this % of target speed to be real
MIN_OSCILLATION_AMPLITUDE_PCT = 5.0
# After increasing Kp, wait this long for transients to die before measuring
SETTLE_AFTER_KP_CHANGE = 1.5  # seconds

OUTPUT_DIR = os.path.expanduser('~')
TIMESTAMP  = datetime.now().strftime('%Y%m%d_%H%M%S')


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
        print(f"\nERROR: Cannot open serial port: {e}")
        print("Make sure ROS is killed first: ~/Autonomous-Mobile-Robot/ws_amr_robot/src/robot_bringup/scripts/tools/kill_robot.sh")
        raise


def send_velocity(ser, v_mmps, w_mradps=0):
    cmd = f"V:{int(v_mmps)},W:{int(w_mradps)}\r\n"
    ser.write(cmd.encode('utf-8'))


def drain_latest(ser):
    """Return most recent valid JSON packet from serial buffer."""
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


def stop_motors(ser, duration=0.5):
    """Send stop command for a given duration."""
    t_end = time.time() + duration
    while time.time() < t_end:
        send_velocity(ser, 0)
        time.sleep(0.05)


# ============================================================
# SPEED MEASUREMENT
# ============================================================

class SpeedMeasurer:
    """
    Maintains running speed estimate from encoder deltas.
    Handles the right wheel polarity flip (polarity_right = -1).
    """
    def __init__(self):
        self.prev_data     = None
        self.prev_time     = None
        self.left_speed    = 0.0
        self.right_speed   = 0.0

    def update(self, data, current_time):
        """Call with new JSON data. Returns (left_speed, right_speed) in mm/s."""
        if self.prev_data is None:
            self.prev_data = data
            self.prev_time = current_time
            return 0.0, 0.0

        dt = current_time - self.prev_time
        if dt < 0.005:
            return self.left_speed, self.right_speed

        left_delta  =  (data['l'] - self.prev_data['l'])
        right_delta = -(data['r'] - self.prev_data['r'])  # polarity_right = -1

        self.left_speed  = (left_delta  * MM_PER_TICK) / dt
        self.right_speed = (right_delta * MM_PER_TICK) / dt

        self.prev_data = data
        self.prev_time = current_time

        return self.left_speed, self.right_speed

    def reset(self, data, current_time):
        self.prev_data   = data
        self.prev_time   = current_time
        self.left_speed  = 0.0
        self.right_speed = 0.0


# ============================================================
# OSCILLATION DETECTOR
# ============================================================

class OscillationDetector:
    """
    Detects sustained oscillation in a speed signal.

    How it works:
    - Keeps a rolling window of speed samples
    - Counts how many times the signal crosses the target value
    - If it crosses enough times, oscillation is detected
    - Measures the average period between crossings
    - Measures the peak-to-peak amplitude

    This is similar to how Klipper detects the oscillation
    in its PID calibration routine.
    """
    def __init__(self, target_speed, window_seconds=3.0):
        self.target        = target_speed
        self.window_size   = int(window_seconds * SAMPLE_RATE_HZ)
        self.samples       = deque(maxlen=self.window_size)
        self.times         = deque(maxlen=self.window_size)
        self.crossing_times = []
        self.last_above    = None

    def add_sample(self, speed, timestamp):
        self.samples.append(speed)
        self.times.append(timestamp)

        # Detect zero crossing of (speed - target)
        above = speed > self.target
        if self.last_above is not None and above != self.last_above:
            self.crossing_times.append(timestamp)
            # Keep only recent crossings (last 10 seconds worth)
            while (self.crossing_times and
                   timestamp - self.crossing_times[0] > 10.0):
                self.crossing_times.pop(0)
        self.last_above = above

    def get_oscillation(self):
        """
        Returns (is_oscillating, period, amplitude) or None.
        is_oscillating: True if sustained oscillation detected
        period: oscillation period in seconds (Pu)
        amplitude: peak-to-peak amplitude in mm/s
        """
        if len(self.crossing_times) < MIN_CROSSINGS_FOR_OSCILLATION:
            return False, None, None

        # Period = average time between crossings * 2
        # (each full oscillation has 2 crossings)
        recent = self.crossing_times[-MIN_CROSSINGS_FOR_OSCILLATION:]
        intervals = [recent[i+1] - recent[i] for i in range(len(recent)-1)]
        half_period = sum(intervals) / len(intervals)
        period = half_period * 2.0

        # Amplitude = peak-to-peak in recent window
        if len(self.samples) < 4:
            return False, None, None

        recent_speeds = list(self.samples)[-int(SAMPLE_RATE_HZ * 2):]
        amplitude = max(recent_speeds) - min(recent_speeds)
        amplitude_pct = (amplitude / abs(self.target)) * 100

        if amplitude_pct < MIN_OSCILLATION_AMPLITUDE_PCT:
            return False, None, None

        # Check that crossings are regular (not random noise)
        if len(intervals) >= 3:
            mean_interval = sum(intervals) / len(intervals)
            variance = sum((x - mean_interval)**2 for x in intervals) / len(intervals)
            cv = math.sqrt(variance) / mean_interval  # coefficient of variation
            if cv > 0.4:
                # Crossings are too irregular, not true oscillation
                return False, None, None

        return True, period, amplitude

    def reset(self):
        self.samples.clear()
        self.times.clear()
        self.crossing_times.clear()
        self.last_above = None


# ============================================================
# RELAY FEEDBACK CONTROLLER
# ============================================================

class RelayController:
    """
    Implements the relay feedback method for PID autotuning.

    Instead of slowly increasing Kp until oscillation (which
    takes many minutes), the relay method forces immediate
    oscillation by acting like an on/off controller with a
    small hysteresis band. This is faster and more reliable.

    How it works:
    - If actual speed < target - hysteresis: output = max_output
    - If actual speed > target + hysteresis: output = 0
    - This creates a bang-bang response that oscillates
    - Measure the oscillation period Pu and amplitude A
    - Ku = 4 * relay_amplitude / (pi * A)  [describing function method]
    - Then apply Ziegler-Nichols formulas

    This is actually MORE accurate than slowly increasing Kp
    because it directly measures the system's natural frequency.
    """
    def __init__(self, target_speed, relay_amplitude,
                 hysteresis_pct=10.0):
        self.target         = target_speed
        self.relay_amp      = relay_amplitude  # output when ON
        self.hysteresis     = abs(target_speed) * (hysteresis_pct / 100.0)
        self.output         = relay_amplitude  # start ON
        self.crossing_times = []
        self.peak_speeds    = []
        self.last_above     = None
        self.phase          = 'rising'  # rising or falling

    def update(self, actual_speed):
        """Update relay state, return output speed command."""
        above = actual_speed > self.target

        if self.last_above is not None:
            if above and not self.last_above:
                # Crossed upward: switch relay OFF
                self.output = 0
                self.crossing_times.append(time.time())
            elif not above and self.last_above:
                # Crossed downward: switch relay ON
                self.output = self.relay_amp
                self.crossing_times.append(time.time())

        self.last_above = above
        return self.output

    def get_results(self, cycles_required=5):
        """
        Returns (success, Ku, Pu) after enough oscillation cycles.
        Returns (False, None, None) if not enough data yet.
        """
        # Need at least 2*cycles crossings (each cycle has 2 crossings)
        if len(self.crossing_times) < cycles_required * 2:
            return False, None, None

        # Use the last cycles_required cycles
        recent = self.crossing_times[-(cycles_required * 2):]
        intervals = [recent[i+1] - recent[i] for i in range(len(recent)-1)]

        # Check regularity of oscillation
        mean_interval = sum(intervals) / len(intervals)
        cv = (math.sqrt(sum((x - mean_interval)**2 for x in intervals) /
              len(intervals))) / mean_interval
        if cv > 0.35:
            # Oscillation not regular enough, need more cycles
            return False, None, None

        # Period = 2 * average half-period
        Pu = mean_interval * 2.0

        # Ku from describing function:
        # For relay with amplitude d and oscillation amplitude a:
        # Ku = 4d / (pi * a)
        # We use relay_amp as d, and the target speed as approximate a
        # (since we're oscillating around the target)
        # This is an approximation. More precise would measure actual amplitude.
        a  = abs(self.target) * 0.15  # approximate: 15% of target = typical amplitude
        Ku = (4.0 * self.relay_amp) / (math.pi * a)

        return True, Ku, Pu


# ============================================================
# ZIEGLER-NICHOLS CALCULATION
# ============================================================

def calculate_zn_gains(Ku, Pu, controller_type='PI'):
    """
    Calculate PID gains from ultimate gain Ku and period Pu
    using Ziegler-Nichols tuning rules.

    controller_type options:
        'P'   : Proportional only
        'PI'  : Proportional + Integral (recommended for constant speed)
        'PID' : Full PID

    For a ground robot at moderate speed, 'PI' is usually best.
    Kd adds noise sensitivity and is usually not needed.
    """
    if controller_type == 'P':
        Kp = 0.50 * Ku
        Ki = 0.0
        Kd = 0.0
    elif controller_type == 'PI':
        Kp = 0.45 * Ku
        Ki = (0.54 * Ku) / Pu
        Kd = 0.0
    else:  # PID
        Kp = 0.60 * Ku
        Ki = (1.20 * Ku) / Pu
        Kd = (0.075 * Ku * Pu)

    return Kp, Ki, Kd


# ============================================================
# SINGLE WHEEL AUTOTUNE
# ============================================================

def autotune_wheel(ser, wheel_name, target_speed, relay_amplitude,
                   cycles_required, verbose=True):
    """
    Run relay feedback autotune on one wheel.
    Returns (Ku, Pu) or raises RuntimeError on failure.

    wheel_name: 'left' or 'right'
    target_speed: desired operating speed in mm/s
    relay_amplitude: the ON-state speed command for the relay
    cycles_required: how many oscillation cycles to measure
    """
    is_left = (wheel_name == 'left')

    if verbose:
        print(f"\n{'='*55}")
        print(f"AUTOTUNING {wheel_name.upper()} WHEEL")
        print(f"  Target speed    : {target_speed} mm/s")
        print(f"  Relay amplitude : {relay_amplitude} mm/s")
        print(f"  Cycles required : {cycles_required}")
        print(f"{'='*55}")

    measurer  = SpeedMeasurer()
    relay     = RelayController(target_speed, relay_amplitude,
                                hysteresis_pct=10.0)

    # Pre-run: let the wheel spin up to roughly target speed
    # so the relay starts near the operating point, not from zero
    if verbose:
        print(f"\n  Spinning up to target speed (2s)...")

    t_spinup = time.time()
    while (time.time() - t_spinup) < 2.0:
        if is_left:
            send_velocity(ser, target_speed, 0)
        else:
            # Only right wheel: send V=0 but we cannot independently
            # command one wheel via V/W format easily.
            # We use a trick: if only right wheel, send a small negative W
            # to make right wheel spin and left nearly stop.
            # Actually the V:W format computes:
            #   left  = V - W/2
            #   right = V + W/2
            # To get left~0 and right~target:
            #   V = target/2, W = -target
            v_cmd = target_speed // 2
            w_cmd = -target_speed
            send_velocity(ser, v_cmd, w_cmd)
        time.sleep(0.05)

    # Flush old data
    ser.reset_input_buffer()
    measurer.reset(None, time.time())

    # Get first clean data point
    deadline = time.time() + 2.0
    first_data = None
    while time.time() < deadline:
        if is_left:
            send_velocity(ser, target_speed, 0)
        data = drain_latest(ser)
        if data is not None and 'l' in data and 'r' in data:
            first_data = data
            measurer.reset(data, time.time())
            break
        time.sleep(0.05)

    if first_data is None:
        raise RuntimeError("No telemetry from STM32 during autotune.")

    if verbose:
        print(f"  Starting relay oscillation. Wheel will oscillate intentionally.")
        print(f"  Waiting for {cycles_required} stable cycles...")

    # ---- MAIN RELAY LOOP ----
    timeout_seconds = 60  # if we cannot get stable oscillation in 60s, give up
    t_start      = time.time()
    log_samples  = []
    last_print   = time.time()
    cycle_count  = 0

    while (time.time() - t_start) < timeout_seconds:
        loop_start = time.time()

        # Get current speed
        data = drain_latest(ser)
        now  = time.time()

        if data is not None and 'l' in data and 'r' in data:
            l_speed, r_speed = measurer.update(data, now)
            actual_speed = l_speed if is_left else r_speed
        else:
            actual_speed = measurer.left_speed if is_left else measurer.right_speed

        # Update relay
        relay_output = relay.update(actual_speed)

        # Send command based on which wheel we are tuning
        if is_left:
            send_velocity(ser, relay_output, 0)
        else:
            v_cmd = relay_output // 2
            w_cmd = -relay_output
            send_velocity(ser, v_cmd, w_cmd)

        # Log
        log_samples.append({
            'time':   now - t_start,
            'actual': actual_speed,
            'output': relay_output,
            'target': target_speed
        })

        # Check if we have enough cycles
        success, Ku, Pu = relay.get_results(cycles_required)
        if success:
            cycle_count = len(relay.crossing_times) // 2
            if verbose:
                print(f"\n  Oscillation detected after {cycle_count} cycles.")
                print(f"  Ku = {Ku:.4f},  Pu = {Pu:.4f} s")
            break

        # Progress update every 3 seconds
        if verbose and (now - last_print) > 3.0:
            crossings = len(relay.crossing_times)
            cycles_so_far = crossings // 2
            print(f"  ... {cycles_so_far}/{cycles_required} cycles  "
                  f"(actual={actual_speed:.0f} mm/s, "
                  f"output={relay_output:.0f})")
            last_print = now

        # Maintain sample rate
        elapsed = time.time() - loop_start
        sleep_time = SAMPLE_DT - elapsed
        if sleep_time > 0:
            time.sleep(sleep_time)

    else:
        # Timeout
        raise RuntimeError(
            f"Autotune timeout for {wheel_name} wheel after {timeout_seconds}s. "
            f"Only got {len(relay.crossing_times)//2} cycles. "
            f"Try increasing relay_amplitude or target_speed.")

    # Stop the wheel
    stop_motors(ser, 0.5)

    return Ku, Pu, log_samples


# ============================================================
# SAVE AND REPORT
# ============================================================

def save_autotune_log(log_left, log_right, filename):
    """Save raw autotune data to CSV for plotting."""
    with open(filename, 'w', newline='') as f:
        writer = csv.writer(f)
        writer.writerow(['wheel', 'time_s', 'actual_mmps',
                         'output_mmps', 'target_mmps'])
        for s in log_left:
            writer.writerow(['left',  round(s['time'],4),
                             round(s['actual'],2),
                             s['output'], s['target']])
        for s in log_right:
            writer.writerow(['right', round(s['time'],4),
                             round(s['actual'],2),
                             s['output'], s['target']])
    print(f"Autotune log saved to: {filename}")


def save_result_config(results, filename):
    """Save the calculated PID values as a C code snippet."""
    with open(filename, 'w') as f:
        f.write("/* ================================================\n")
        f.write(" * PID AUTOTUNE RESULTS\n")
        f.write(f" * Generated: {datetime.now().strftime('%Y-%m-%d %H:%M:%S')}\n")
        f.write(" *\n")
        f.write(" * Copy these values into Motor_Init() in motor_control.c\n")
        f.write(" * ================================================ */\n\n")

        for wheel, data in results.items():
            f.write(f"/* {wheel.upper()} WHEEL */\n")
            f.write(f"/* Ku={data['Ku']:.4f}, Pu={data['Pu']:.4f}s, "
                    f"method=ZN-PI */\n")
            f.write(f"g_pid_{wheel}.kp = {data['Kp']:.4f}f;\n")
            f.write(f"g_pid_{wheel}.ki = {data['Ki']:.4f}f;\n")
            f.write(f"g_pid_{wheel}.kd = {data['Kd']:.4f}f;\n\n")

    print(f"Config saved to: {filename}")


def plot_autotune(log_left, log_right, target_speed, png_filename):
    """Plot the relay oscillation data."""
    try:
        import matplotlib
        matplotlib.use('Agg')
        import matplotlib.pyplot as plt
    except ImportError:
        print("matplotlib not installed. Skipping plot.")
        print("Install: pip3 install matplotlib --break-system-packages")
        return

    fig, axes = plt.subplots(2, 1, figsize=(14, 8), sharex=False)
    fig.suptitle(f'PID Autotune - Relay Feedback Method\n'
                 f'Target: {target_speed} mm/s',
                 fontsize=13, fontweight='bold')

    datasets = [('LEFT',  log_left,  axes[0], '#2196F3'),
                ('RIGHT', log_right, axes[1], '#F44336')]

    for label, log, ax, color in datasets:
        if not log:
            continue
        times   = [s['time']   for s in log]
        actuals = [s['actual'] for s in log]
        outputs = [s['output'] for s in log]

        ax.plot(times, actuals, color=color, linewidth=1.5,
                label='Actual Speed', alpha=0.8)
        ax.step(times, outputs, color='gray', linewidth=1,
                label='Relay Output', alpha=0.5, where='post')
        ax.axhline(y=target_speed, color='green', linewidth=1.5,
                   linestyle='--', label='Target', alpha=0.8)
        ax.axhline(y=0, color='black', linewidth=0.5, alpha=0.3)

        ax.set_title(f'{label} Wheel Relay Oscillation', fontsize=11)
        ax.set_ylabel('Speed (mm/s)')
        ax.legend(loc='upper right', fontsize=9)
        ax.grid(True, alpha=0.3)

    axes[-1].set_xlabel('Time (seconds)')
    plt.tight_layout()
    plt.savefig(png_filename, dpi=150, bbox_inches='tight')
    print(f"Plot saved to: {png_filename}")


# ============================================================
# MAIN
# ============================================================

def main():
    parser = argparse.ArgumentParser(
        description='PID Autotune for AMR Robot Wheels (Relay Feedback Method)')
    parser.add_argument('--target-speed', type=int,
                        default=DEFAULT_TARGET_SPEED,
                        help=f'Operating speed in mm/s (default: {DEFAULT_TARGET_SPEED})')
    parser.add_argument('--relay-amplitude', type=int, default=None,
                        help='Relay ON amplitude in mm/s (default: 1.5x target)')
    parser.add_argument('--cycles', type=int, default=DEFAULT_CYCLES,
                        help=f'Oscillation cycles to measure (default: {DEFAULT_CYCLES})')
    parser.add_argument('--left-only',  action='store_true')
    parser.add_argument('--right-only', action='store_true')
    args = parser.parse_args()

    target_speed = args.target_speed

    # Relay amplitude: how hard we push during ON phase
    # Should be higher than deadband to ensure movement
    # Default: 1.5x target, minimum 400 mm/s to overcome deadband
    relay_amplitude = args.relay_amplitude
    if relay_amplitude is None:
        relay_amplitude = max(400, int(target_speed * 1.5))
        relay_amplitude = min(relay_amplitude, 1000)  # cap at 1000 mm/s

    tune_left  = not args.right_only
    tune_right = not args.left_only

    print("=" * 55)
    print("PID AUTOTUNE  -  Relay Feedback Method")
    print("=" * 55)
    print(f"Target speed    : {target_speed} mm/s  "
          f"({target_speed/1000:.2f} m/s)")
    print(f"Relay amplitude : {relay_amplitude} mm/s")
    print(f"Cycles required : {args.cycles}")
    print(f"Tune left       : {tune_left}")
    print(f"Tune right      : {tune_right}")
    print()
    print("REQUIREMENTS:")
    print("  1. Robot wheels MUST be off the ground")
    print("  2. ROS must be killed: ~/Autonomous-Mobile-Robot/ws_amr_robot/src/robot_bringup/scripts/tools/kill_robot.sh")
    print("  3. STM32 must have PID firmware (not characterization)")
    print()
    print("The wheels will OSCILLATE during this test.")
    print("This is normal. Do not be alarmed.")
    print()

    input("Press ENTER to start, Ctrl+C to cancel...")

    ser = connect_serial()
    results  = {}
    log_left  = []
    log_right = []

    try:
        if tune_left:
            Ku_l, Pu_l, log_left = autotune_wheel(
                ser, 'left', target_speed, relay_amplitude, args.cycles)

            Kp_l, Ki_l, Kd_l = calculate_zn_gains(Ku_l, Pu_l, 'PI')
            results['left'] = {
                'Ku': Ku_l, 'Pu': Pu_l,
                'Kp': Kp_l, 'Ki': Ki_l, 'Kd': Kd_l
            }

            print(f"\n  LEFT WHEEL RESULTS:")
            print(f"    Ku = {Ku_l:.4f},  Pu = {Pu_l:.3f} s")
            print(f"    Kp = {Kp_l:.4f}")
            print(f"    Ki = {Ki_l:.4f}")
            print(f"    Kd = {Kd_l:.4f}  (set to 0 if not needed)")

            # Wait between wheels
            print(f"\n  Resting 3 seconds before right wheel test...")
            stop_motors(ser, 3.0)

        if tune_right:
            Ku_r, Pu_r, log_right = autotune_wheel(
                ser, 'right', target_speed, relay_amplitude, args.cycles)

            Kp_r, Ki_r, Kd_r = calculate_zn_gains(Ku_r, Pu_r, 'PI')
            results['right'] = {
                'Ku': Ku_r, 'Pu': Pu_r,
                'Kp': Kp_r, 'Ki': Ki_r, 'Kd': Kd_r
            }

            print(f"\n  RIGHT WHEEL RESULTS:")
            print(f"    Ku = {Ku_r:.4f},  Pu = {Pu_r:.3f} s")
            print(f"    Kp = {Kp_r:.4f}")
            print(f"    Ki = {Ki_r:.4f}")
            print(f"    Kd = {Kd_r:.4f}  (set to 0 if not needed)")

    except RuntimeError as e:
        print(f"\nAutotuning failed: {e}")
        print("Possible causes:")
        print("  - Relay amplitude too low (wheel not spinning)")
        print("  - Target speed too low (near deadband)")
        print("  - STM32 PID is overriding the relay (disable PID for this test)")
        print("  Try: python3 ~/Autonomous-Mobile-Robot/ws_amr_robot/src/robot_bringup/scripts/characterization/pid_autotune.py --target-speed 400 --relay-amplitude 700")
    except KeyboardInterrupt:
        print("\nInterrupted by user.")
    finally:
        stop_motors(ser, 0.5)
        ser.close()
        print("\nMotors stopped. Serial closed.")

    if not results:
        print("No results to save.")
        return

    # Print final summary with exact code to copy
    print("\n" + "=" * 55)
    print("FINAL RESULTS - COPY INTO motor_control.c")
    print("=" * 55)
    print("Replace the values in Motor_Init() with these:\n")
    for wheel, data in results.items():
        print(f"    /* {wheel.upper()} WHEEL  (Ku={data['Ku']:.3f}, "
              f"Pu={data['Pu']:.3f}s) */")
        print(f"    g_pid_{wheel}.kp = {data['Kp']:.4f}f;")
        print(f"    g_pid_{wheel}.ki = {data['Ki']:.4f}f;")
        print(f"    g_pid_{wheel}.kd = 0.0f;  /* start with 0, add only if needed */")
        print()

    print("NOTE: These are Ziegler-Nichols PI starting values.")
    print("They give good response but may need small manual")
    print("adjustment. Use pid_step_response.py to verify.")

    # Save files
    log_file = os.path.join(OUTPUT_DIR, f'autotune_log_{TIMESTAMP}.csv')
    cfg_file = os.path.join(OUTPUT_DIR, f'autotune_result_{TIMESTAMP}.txt')
    png_file = os.path.join(OUTPUT_DIR, f'autotune_plot_{TIMESTAMP}.png')

    save_autotune_log(log_left, log_right, log_file)
    save_result_config(results, cfg_file)
    plot_autotune(log_left, log_right, target_speed, png_file)

    print(f"\nFiles saved:")
    print(f"  Config : {cfg_file}")
    print(f"  Log    : {log_file}")
    print(f"  Plot   : {png_file}")


if __name__ == '__main__':
    main()
