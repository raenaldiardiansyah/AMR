#!/usr/bin/env python3
"""
collect_pid_comparison.py -- PID step response test for prof report
-------------------------------------------------------------------
Sends a speed target via V: command, records both wheel speeds,
computes step response metrics, and generates a comparison-ready plot.

Tests multiple target speeds, records time-series of L/R speed,
then reports: rise time, overshoot, settling time, steady-state error,
L-R symmetry.

Can test with different gain sets (via K: command if firmware supports it).

USAGE:
  # With ROS stack running (uses V: through stm32_bridge):
  python3 collect_pid_comparison.py --label prof_original

  # Direct serial (ROS must be stopped):
  python3 collect_pid_comparison.py --serial --label corrected_gains
"""

import argparse
import math
import os
import sys
import time
import json
import serial

sys.path.insert(0, os.path.dirname(__file__))
import amr_test_utils as U

SERIAL_PORT = '/dev/serial/by-id/usb-FTDI_FT232R_USB_UART_A5069RR4-if00-port0'
BAUD_RATE = 115200
TICKS_PER_REV = 2400.0
MM_PER_TICK = (math.pi * 150.0) / TICKS_PER_REV
DEFAULT_TARGETS = [100, 150, 200]  # mm/s
REPS = 5
STEP_DURATION = 20.0   # seconds per step
STILL_BEFORE = 2.0    # seconds still before step


def wrap16(d):
    d %= 65536
    if d >= 32768:
        d -= 65536
    return d


class SerialCollector:
    def __init__(self, port, baud):
        self.ser = serial.Serial(port, baud, timeout=0.15)
        self.ser.reset_input_buffer()
        time.sleep(0.5)
        self.ser.reset_input_buffer()
        self.prev_pkt = None

    def send_velocity(self, v_mmps, w_mradps=0):
        cmd = f"V:{int(v_mmps)},W:{int(w_mradps)}\r\n"
        self.ser.write(cmd.encode())

    def send_gains(self, kp, ki, kd):
        """Send K: command if firmware supports it."""
        kp100 = int(kp * 100)
        ki100 = int(ki * 100)
        kd100 = int(kd * 100)
        cmd = f"K:{kp100},{ki100},{kd100}\r\n"
        self.ser.write(cmd.encode())
        time.sleep(0.1)

    def read_speeds(self):
        """Read packets, return list of (timestamp, left_mmps, right_mmps, fw_dt_ms)."""
        results = []
        while self.ser.in_waiting > 0:
            try:
                line = self.ser.readline().decode('utf-8', errors='ignore').strip()
                if line.startswith('{') and line.endswith('}'):
                    pkt = json.loads(line)
                    if 'l' in pkt and 'r' in pkt and self.prev_pkt is not None:
                        l_delta = wrap16(pkt['l'] - self.prev_pkt['l'])
                        r_delta = -wrap16(pkt['r'] - self.prev_pkt['r'])
                        fw_dt = pkt.get('dt', 52) / 1000.0
                        l_mmps = l_delta * MM_PER_TICK / fw_dt
                        r_mmps = r_delta * MM_PER_TICK / fw_dt
                        results.append((time.time(), l_mmps, r_mmps, fw_dt))
                    self.prev_pkt = pkt
            except Exception:
                pass
        return results

    def stop(self):
        for _ in range(5):
            self.send_velocity(0, 0)
            time.sleep(0.05)

    def close(self):
        self.stop()
        self.ser.close()


def compute_metrics(times, speeds, target):
    """Compute step response metrics from time-series."""
    if not speeds or target == 0:
        return {}

    # Find step start (first non-zero target region)
    ss_start = int(len(speeds) * 0.6)
    ss_speeds = speeds[ss_start:]

    if not ss_speeds:
        return {}

    ss_mean = sum(ss_speeds) / len(ss_speeds)
    ss_std = (sum((s - ss_mean)**2 for s in ss_speeds) / len(ss_speeds)) ** 0.5
    ss_error = target - ss_mean
    ss_error_pct = ss_error / target * 100

    overshoot = max(speeds) - target
    overshoot_pct = (overshoot / target * 100) if overshoot > 0 else 0.0

    # Rise time: first time reaching 90% of target
    threshold_90 = target * 0.9
    t0 = times[0]
    rise_time = None
    for t, s in zip(times, speeds):
        if s >= threshold_90:
            rise_time = t - t0
            break

    # Settling time: last time outside ±5% of steady state
    band = target * 0.05
    settle_time = None
    for i in range(len(speeds) - 1, -1, -1):
        if abs(speeds[i] - ss_mean) > band:
            settle_time = times[i] - t0
            break

    return {
        'ss_speed': round(ss_mean, 1),
        'ss_std': round(ss_std, 1),
        'ss_error_mmps': round(ss_error, 1),
        'ss_error_pct': round(ss_error_pct, 1),
        'overshoot_pct': round(overshoot_pct, 1),
        'rise_time_s': round(rise_time, 3) if rise_time else None,
        'settle_time_s': round(settle_time, 3) if settle_time else None,
    }


def run_step_test(collector, target_mmps, duration, still_before):
    """Run one step response: still -> step -> still. Returns time-series."""
    samples = []

    # Still phase
    collector.ser.reset_input_buffer()
    collector.prev_pkt = None
    collector.send_velocity(0, 0)
    time.sleep(0.2)
    collector.ser.reset_input_buffer()
    collector.prev_pkt = None

    # Seed prev_pkt
    t_wait = time.time() + 1.0
    while collector.prev_pkt is None and time.time() < t_wait:
        collector.send_velocity(0, 0)
        collector.read_speeds()
        time.sleep(0.02)

    t_zero = time.time()
    t_end = t_zero + still_before
    while time.time() < t_end:
        collector.send_velocity(0, 0)
        for ts, l, r, dt in collector.read_speeds():
            samples.append({
                't': round(ts - t_zero, 4),
                'phase': 'still',
                'target_mmps': 0,
                'left_mmps': round(l, 2),
                'right_mmps': round(r, 2),
            })
        time.sleep(0.02)

    # Step phase
    t_step = time.time()
    t_end = t_step + duration
    last_print = 0
    while time.time() < t_end:
        collector.send_velocity(target_mmps, 0)
        for ts, l, r, dt in collector.read_speeds():
            samples.append({
                't': round(ts - t_zero, 4),
                'phase': 'step',
                'target_mmps': target_mmps,
                'left_mmps': round(l, 2),
                'right_mmps': round(r, 2),
            })
        now = time.time()
        if now - last_print > 1.0:
            remain = t_end - now
            if samples:
                s = samples[-1]
                print(f"\r    {remain:.0f}s  L={s['left_mmps']:.0f} R={s['right_mmps']:.0f} "
                      f"target={target_mmps}   ", end='', flush=True)
            last_print = now
        time.sleep(0.02)

    # Stop
    collector.stop()
    print()
    return samples


def plot_comparison(all_results, out_path):
    """Generate step response comparison plot."""
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt

    targets = sorted(set(r['target'] for r in all_results))
    n_targets = len(targets)

    fig, axes = plt.subplots(n_targets, 1, figsize=(14, 4 * n_targets), sharex=False)
    if n_targets == 1:
        axes = [axes]

    for ax, target in zip(axes, targets):
        target_data = [r for r in all_results if r['target'] == target]
        for rep_data in target_data:
            times = [s['t'] for s in rep_data['samples']]
            l_spd = [s['left_mmps'] for s in rep_data['samples']]
            r_spd = [s['right_mmps'] for s in rep_data['samples']]
            rep = rep_data['rep']
            alpha = 0.6 if rep > 1 else 1.0
            ax.plot(times, l_spd, color='#1565C0', linewidth=1.2, alpha=alpha,
                    label='Left' if rep == 1 else '')
            ax.plot(times, r_spd, color='#C62828', linewidth=1.2, alpha=alpha,
                    label='Right' if rep == 1 else '')

        ax.axhline(y=target, color='green', linestyle='--', linewidth=1.5,
                   label=f'Target {target} mm/s')
        ax.set_ylabel('Speed (mm/s)')
        ax.set_title(f'Step Response: target = {target} mm/s')
        ax.legend(loc='lower right')
        ax.grid(True, alpha=0.3)

    axes[-1].set_xlabel('Time (s)')
    plt.tight_layout()
    plt.savefig(out_path, dpi=150, bbox_inches='tight')
    plt.close()


def main():
    ap = argparse.ArgumentParser(description="PID step response comparison test")
    ap.add_argument('--targets', default=None,
                    help="Comma-separated target speeds in mm/s (default: 100,150,200)")
    ap.add_argument('--reps', type=int, default=5)
    ap.add_argument('--duration', type=float, default=STEP_DURATION)
    ap.add_argument('--label', default=None, help="Label for output files")
    ap.add_argument('--gains', default=None,
                    help="Set gains via K: command: 'kp,ki,kd' (e.g. '6.41,84.02,0')")
    ap.add_argument('--serial', action='store_true',
                    help="Use direct serial (ROS must be stopped)")
    args = ap.parse_args()

    targets = [int(x) for x in args.targets.split(',')] if args.targets else DEFAULT_TARGETS

    print("=" * 64)
    print(" PID STEP RESPONSE TEST")
    print(f" Targets: {targets} mm/s")
    print(f" Reps: {args.reps} per target")
    print(f" Duration: {args.duration}s per step")
    if args.gains:
        print(f" Gains: {args.gains}")
    if args.label:
        print(f" Label: {args.label}")
    print("=" * 64)

    collector = SerialCollector(SERIAL_PORT, BAUD_RATE)

    if args.gains:
        parts = [float(x) for x in args.gains.split(',')]
        kp, ki, kd = parts[0], parts[1], parts[2] if len(parts) > 2 else 0
        print(f"\n  Sending gains: Kp={kp} Ki={ki} Kd={kd}")
        collector.send_gains(kp, ki, kd)
        time.sleep(0.5)

    all_results = []
    all_samples = []

    try:
        for target in targets:
            for rep in range(1, args.reps + 1):
                print(f"\n  [{target} mm/s] Rep {rep}/{args.reps}")
                samples = run_step_test(collector, target, args.duration, STILL_BEFORE)

                step_samples = [s for s in samples if s['phase'] == 'step']
                if step_samples:
                    l_times = [s['t'] for s in step_samples]
                    l_speeds = [s['left_mmps'] for s in step_samples]
                    r_speeds = [s['right_mmps'] for s in step_samples]

                    l_metrics = compute_metrics(l_times, l_speeds, target)
                    r_metrics = compute_metrics(l_times, r_speeds, target)

                    print(f"    Left:  SS={l_metrics.get('ss_speed','-')} mm/s  "
                          f"err={l_metrics.get('ss_error_pct','-')}%  "
                          f"rise={l_metrics.get('rise_time_s','-')}s  "
                          f"overshoot={l_metrics.get('overshoot_pct','-')}%")
                    print(f"    Right: SS={r_metrics.get('ss_speed','-')} mm/s  "
                          f"err={r_metrics.get('ss_error_pct','-')}%  "
                          f"rise={r_metrics.get('rise_time_s','-')}s  "
                          f"overshoot={r_metrics.get('overshoot_pct','-')}%")

                    lr_diff = abs(l_metrics.get('ss_speed', 0) - r_metrics.get('ss_speed', 0))
                    print(f"    L-R diff: {lr_diff:.1f} mm/s")

                all_results.append({
                    'target': target,
                    'rep': rep,
                    'samples': samples,
                    'left_metrics': l_metrics if step_samples else {},
                    'right_metrics': r_metrics if step_samples else {},
                })

                for s in samples:
                    s['target_mmps'] = target
                    s['rep'] = rep
                all_samples.extend(samples)

                time.sleep(1.0)

    except KeyboardInterrupt:
        print("\nInterrupted.")
    finally:
        collector.close()

    if not all_samples:
        print("No data collected.")
        return

    # Save
    out_dir = os.path.expanduser('~/thesis_data/pid_comparison')
    os.makedirs(out_dir, exist_ok=True)
    from datetime import datetime
    ts = datetime.now().strftime('%Y%m%d_%H%M%S')
    lbl = f"_{args.label}" if args.label else ""

    csv_path = os.path.join(out_dir, f"pid_step{lbl}_{ts}.csv")
    U.save_csv(csv_path,
               ['target_mmps', 'rep', 't', 'phase', 'left_mmps', 'right_mmps'],
               all_samples)

    # Metrics summary CSV
    metrics_rows = []
    for r in all_results:
        for wheel, m in [('left', r.get('left_metrics', {})),
                         ('right', r.get('right_metrics', {}))]:
            if m:
                metrics_rows.append({
                    'target': r['target'], 'rep': r['rep'], 'wheel': wheel,
                    **m
                })

    metrics_path = os.path.join(out_dir, f"pid_metrics{lbl}_{ts}.csv")
    if metrics_rows:
        U.save_csv(metrics_path,
                   ['target', 'rep', 'wheel', 'ss_speed', 'ss_std',
                    'ss_error_mmps', 'ss_error_pct', 'overshoot_pct',
                    'rise_time_s', 'settle_time_s'],
                   metrics_rows)

    # Plot
    png_path = os.path.join(out_dir, f"pid_step{lbl}_{ts}.png")
    try:
        plot_comparison(all_results, png_path)
        print(f"\n  Plot: {png_path}")
    except Exception as e:
        print(f"\n  Plot failed: {e}")

    # Print summary table
    print(f"\n{'=' * 64}")
    print(f" SUMMARY{' - ' + args.label if args.label else ''}")
    print(f"{'=' * 64}")
    print(f" {'target':>7} | {'wheel':>6} | {'SS speed':>9} | {'SS err%':>8} | "
          f"{'rise(s)':>8} | {'overshoot%':>10}")
    print(" " + "-" * 60)

    for target in targets:
        for wheel in ['left', 'right']:
            rows = [r for r in metrics_rows
                    if r['target'] == target and r['wheel'] == wheel]
            if rows:
                avg_ss = sum(r['ss_speed'] for r in rows) / len(rows)
                avg_err = sum(r['ss_error_pct'] for r in rows) / len(rows)
                rises = [r['rise_time_s'] for r in rows if r['rise_time_s'] is not None]
                avg_rise = sum(rises) / len(rises) if rises else float('nan')
                avg_os = sum(r['overshoot_pct'] for r in rows) / len(rows)
                print(f" {target:>7} | {wheel:>6} | {avg_ss:9.1f} | {avg_err:+7.1f}% | "
                      f"{avg_rise:8.3f} | {avg_os:9.1f}%")

    print(f"\n  Raw CSV:     {csv_path}")
    print(f"  Metrics CSV: {metrics_path}")


if __name__ == '__main__':
    main()
