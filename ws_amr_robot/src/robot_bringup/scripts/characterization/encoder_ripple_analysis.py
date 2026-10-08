#!/usr/bin/env python3
"""
ENCODER RIPPLE FREQUENCY ANALYSIS
====================================
Thesis: Kendali dan Navigasi AMR Berbasis LiDAR dan IMU di Warehouse
Author: Reinald Mariel - Universitas Telkom

PURPOSE:
  Your professor noticed ripple (noise) in encoder speed readings at steady state.
  This script:
  1. Runs the robot at a constant speed
  2. Records the raw encoder speed for 20 seconds
  3. Does an FFT (frequency analysis) on the signal
  4. Finds where the ripple frequency is
  5. Saves data for you to plot and report in your thesis

WHY THIS MATTERS:
  The encoder has 2400 ticks/revolution. At high speed, each rotation takes
  a short time. The time between ticks has tiny variations due to mechanical
  imperfections. When you calculate speed from ticks/time, these variations
  show up as repeating noise (ripple) at a specific frequency.
  
  Once you know the ripple frequency, you can design a low-pass filter
  that cuts off at a frequency below the ripple. This cleans the signal
  so your PID gets smooth speed feedback.

HOW TO USE:
  Terminal 1:
    ~/Autonomous-Mobile-Robot/ws_amr_robot/src/robot_bringup/scripts/tools/kill_robot.sh && ros2 launch robot_bringup hardware_launch.py

  Terminal 2:
    source ~/Autonomous-Mobile-Robot/ws_amr_robot/install/setup.bash
    python3 ~/Autonomous-Mobile-Robot/ws_amr_robot/src/robot_bringup/scripts/characterization/encoder_ripple_analysis.py

  Run at multiple speeds. 100, 150, 200 mm/s separately.
  The ripple frequency will be different at each speed.

INSTALL REQUIREMENT (run once):
  pip3 install numpy matplotlib --break-system-packages
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import csv
import os
import time
import datetime
import math

# ============================================================
# SETTINGS
# ============================================================
TEST_SPEED_MMPS   = 150       # Try 100, 150, 200 mm/s
RECORD_DURATION   = 20.0      # Seconds to record (need long enough for FFT)
WARMUP_DURATION   = 3.0       # Seconds at speed before recording starts
SAMPLE_RATE_HZ    = 100       # Record at 100 Hz to capture high freq ripple

OUTPUT_DIR = os.path.expanduser('~/thesis_data/ripple_analysis')


class RippleAnalyzer(Node):
    def __init__(self):
        super().__init__('ripple_analyzer')

        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        self.actual_speed = 0.0
        self.speed_history = []
        self.recording = False
        self.record_start = None

        # Subscribe to odom_raw for real encoder speed
        self.sub_odom = self.create_subscription(
            Odometry, '/odom_raw', self._cb_odom, 10)

        self.get_logger().info('Ripple Analyzer ready.')

    def _cb_odom(self, msg):
        spd = msg.twist.twist.linear.x * 1000.0    # m/s to mm/s
        self.actual_speed = spd
        if self.recording:
            t = time.time() - self.record_start
            self.speed_history.append((t, spd))

    def send_speed(self, speed_mmps):
        msg = Twist()
        msg.linear.x = speed_mmps / 1000.0
        msg.angular.z = 0.0
        self.cmd_pub.publish(msg)

    def stop(self):
        self.send_speed(0)


def do_fft_analysis(speed_data, sample_rate):
    """
    Does FFT on the speed signal to find dominant frequency components.
    Returns list of (frequency_hz, amplitude) sorted by amplitude descending.
    """
    try:
        import numpy as np
        speeds = np.array([s for _, s in speed_data])
        # Remove DC offset (mean) so we see only the oscillating part
        speeds_ac = speeds - np.mean(speeds)
        n = len(speeds_ac)
        # FFT
        fft_result = np.fft.rfft(speeds_ac)
        fft_mag = np.abs(fft_result) * 2 / n
        freqs = np.fft.rfftfreq(n, d=1.0/sample_rate)
        # Build list of (freq, amplitude) pairs, skip DC (index 0)
        freq_amp = [(freqs[i], fft_mag[i]) for i in range(1, len(freqs))]
        # Sort by amplitude, highest first
        freq_amp.sort(key=lambda x: x[1], reverse=True)
        return freq_amp[:20]    # Return top 20 frequencies
    except ImportError:
        print('[WARNING] numpy not installed. Skipping FFT analysis.')
        print('Run: pip3 install numpy --break-system-packages')
        return []


def estimate_expected_ripple_freq(speed_mmps):
    """
    Calculate theoretically expected ripple frequency based on robot specs.
    Speed in mm/s, wheel circumference = pi * 150mm = 471.2mm
    Encoder ticks per revolution = 2400
    
    At speed V mm/s:
      Revolutions per second = V / circumference
      Ripple frequency = revolutions/s * ticks_per_rev
      But this is the tick rate, not the ripple period.
      
    Actually the ripple from gear teeth or encoder slots:
      Wheel RPM = (V * 60) / (pi * diameter_mm)
    """
    wheel_circ_mm = math.pi * 150.0    # pi * diameter
    revs_per_sec = speed_mmps / wheel_circ_mm
    # Ripple from encoder slots: 2400 ticks/rev
    # But actual observable ripple is usually at lower harmonics
    tick_rate_hz = revs_per_sec * 2400
    return revs_per_sec, tick_rate_hz


def main():
    rclpy.init()
    node = RippleAnalyzer()

    os.makedirs(OUTPUT_DIR, exist_ok=True)
    ts = datetime.datetime.now().strftime('%Y%m%d_%H%M%S')
    raw_csv = os.path.join(OUTPUT_DIR, f'ripple_raw_{TEST_SPEED_MMPS}mmps_{ts}.csv')
    fft_csv = os.path.join(OUTPUT_DIR, f'ripple_fft_{TEST_SPEED_MMPS}mmps_{ts}.csv')

    print('=' * 55)
    print('ENCODER RIPPLE ANALYSIS')
    print('=' * 55)
    print(f'Test speed    : {TEST_SPEED_MMPS} mm/s')
    print(f'Record time   : {RECORD_DURATION} s')
    print(f'Sample rate   : {SAMPLE_RATE_HZ} Hz')
    print()

    revs_per_sec, tick_rate = estimate_expected_ripple_freq(TEST_SPEED_MMPS)
    print(f'Theoretical wheel speed   : {revs_per_sec:.2f} rev/s')
    print(f'Expected tick rate        : {tick_rate:.0f} Hz')
    print(f'  (Actual observable ripple is usually a SUBharmonic of this)')
    print(f'  Look for peaks at: {revs_per_sec:.1f} Hz, {revs_per_sec*2:.1f} Hz, {revs_per_sec*4:.1f} Hz')
    print()
    print('MAKE SURE:')
    print('  - Robot is on flat ground with at least 2m of clear space')
    print('  - Only hardware_launch.py is running (NO navigation)')
    print()
    input('Press ENTER to start...')

    # Warmup: run at speed but don't record yet
    print(f'[1/3] Warming up at {TEST_SPEED_MMPS} mm/s for {WARMUP_DURATION}s...')
    warmup_end = time.time() + WARMUP_DURATION
    while time.time() < warmup_end:
        node.send_speed(TEST_SPEED_MMPS)
        rclpy.spin_once(node, timeout_sec=0.01)

    # Recording phase
    print(f'[2/3] Recording for {RECORD_DURATION}s...')
    node.recording = True
    node.record_start = time.time()
    record_end = time.time() + RECORD_DURATION
    while time.time() < record_end:
        node.send_speed(TEST_SPEED_MMPS)
        rclpy.spin_once(node, timeout_sec=0.005)

    node.recording = False
    node.stop()

    # Spin briefly to process remaining callbacks
    stop_end = time.time() + 1.0
    while time.time() < stop_end:
        rclpy.spin_once(node, timeout_sec=0.01)

    speed_data = node.speed_history
    print(f'[3/3] Collected {len(speed_data)} samples.')

    if len(speed_data) < 100:
        print('[ERROR] Too few samples collected. Is odom_raw topic publishing?')
        print('Check: ros2 topic hz /odom_raw')
        node.destroy_node()
        rclpy.shutdown()
        return

    # Save raw data
    with open(raw_csv, 'w', newline='') as f:
        writer = csv.writer(f)
        writer.writerow(['time_s', 'speed_mmps'])
        for t, s in speed_data:
            writer.writerow([round(t, 4), round(s, 3)])
    print(f'Raw data saved: {raw_csv}')

    # FFT analysis
    print()
    print('Running FFT frequency analysis...')
    actual_sample_rate = len(speed_data) / RECORD_DURATION
    print(f'Actual sample rate: {actual_sample_rate:.1f} Hz')

    fft_results = do_fft_analysis(speed_data, actual_sample_rate)

    if fft_results:
        print()
        print('TOP FREQUENCY COMPONENTS (sorted by amplitude):')
        print(f'{"Frequency (Hz)":<20} {"Amplitude (mm/s)":<20} {"Interpretation"}')
        print('-' * 65)
        for freq, amp in fft_results[:10]:
            if amp < 0.5:
                continue    # Skip tiny components
            interp = ''
            if abs(freq - revs_per_sec) < 0.5:
                interp = '<-- Wheel rotation frequency'
            elif abs(freq - revs_per_sec*2) < 0.5:
                interp = '<-- 2nd harmonic of wheel'
            elif freq < 5.0:
                interp = '<-- Low freq (robot body motion)'
            elif freq > 50.0:
                interp = '<-- High freq (electrical noise)'
            print(f'{freq:<20.2f} {amp:<20.3f} {interp}')

        # Save FFT data
        with open(fft_csv, 'w', newline='') as f:
            writer = csv.writer(f)
            writer.writerow(['frequency_hz', 'amplitude_mmps'])
            for freq, amp in fft_results:
                writer.writerow([round(freq, 3), round(amp, 4)])
        print(f'\nFFT data saved: {fft_csv}')

    # Summary stats
    speeds = [s for _, s in speed_data]
    avg_speed = sum(speeds) / len(speeds)
    max_speed = max(speeds)
    min_speed = min(speeds)
    speed_range = max_speed - min_speed
    error_pct = abs(TEST_SPEED_MMPS - avg_speed) / TEST_SPEED_MMPS * 100

    print()
    print('=' * 55)
    print('RIPPLE SUMMARY')
    print('=' * 55)
    print(f'Command speed         : {TEST_SPEED_MMPS} mm/s')
    print(f'Average actual speed  : {avg_speed:.1f} mm/s')
    print(f'Steady-state error    : {error_pct:.1f}%')
    print(f'Max speed             : {max_speed:.1f} mm/s')
    print(f'Min speed             : {min_speed:.1f} mm/s')
    print(f'Ripple range (pk-pk)  : {speed_range:.1f} mm/s')
    print(f'Ripple as % of cmd    : {speed_range/TEST_SPEED_MMPS*100:.1f}%')
    print()
    print('WHAT TO WRITE IN THESIS:')
    print(f'  At {TEST_SPEED_MMPS} mm/s, encoder speed shows peak-to-peak ripple')
    print(f'  of {speed_range:.1f} mm/s ({speed_range/TEST_SPEED_MMPS*100:.1f}% of command).')
    if fft_results and fft_results[0][1] > 0.5:
        dominant_freq = fft_results[0][0]
        print(f'  FFT shows dominant noise component at {dominant_freq:.1f} Hz.')
        lp_cutoff = dominant_freq * 0.5
        print(f'  Recommended low-pass filter cutoff: {lp_cutoff:.1f} Hz')
        print(f'  (Below the ripple, above the actual motion bandwidth ~2-5 Hz)')
    print('=' * 55)
    print()
    print('COPY THESE FILES TO YOUR PC FOR PLOTTING:')
    print(f'  {raw_csv}')
    if fft_results:
        print(f'  {fft_csv}')
    print()
    print('ALSO RUN THIS ANALYSIS AT 100 mm/s AND 200 mm/s')
    print('so your thesis shows how ripple frequency scales with speed.')

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
