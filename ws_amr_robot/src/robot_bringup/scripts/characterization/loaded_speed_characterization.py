#!/usr/bin/env python3
"""
loaded_speed_characterization.py
--------------------------------
RAW open-loop motor characterization (ELEVATED, both wheels). Captures BOTH:
  1) STEADY-STATE speed vs PWM  (one settled value per PWM, in rpm and cm/s)
  2) The SETTLING over time      (speed vs time at each PWM, showing the motor
     rise from 0 and flatten to steady state)

Requires the unified firmware that accepts "P:LEFT,RIGHT" raw PWM and streams
{"l":..,"r":..} telemetry from the (working) encoder hardware.

A background thread reads serial nonstop (no lost packets). Speeds come from
wrap-corrected tick deltas. Units use 2400 ticks/rev.

HOW TO USE:
  Terminal 1: flash the unified firmware.
  Terminal 2:
    ~/kill_robot.sh
    python3 .../characterization/loaded_speed_characterization.py
  ELEVATE the robot, wheels free, fingers clear. Ctrl-C aborts and saves.

OUTPUTS in ~/thesis_data/PID_tune_STM32/ :
  RAW_openloop_elevated_<ts>.csv          steady-state vs PWM
  RAW_openloop_elevated_<ts>.png          steady-state characteristic plot
  RAW_openloop_elevated_<ts>_timeseries.csv   speed vs time per PWM
  RAW_openloop_elevated_<ts>_settling.png     settling plot (selected PWMs)
"""

import os
import csv
import math
import time
import json
import threading
import statistics
import serial
from datetime import datetime

SERIAL_PORT = '/dev/serial/by-id/usb-FTDI_FT232R_USB_UART_A5069RR4-if00-port0'
BAUD_RATE   = 115200

WHEEL_DIAMETER = 0.150
TICKS_PER_REV  = 2400.0
M_PER_TICK     = (math.pi * WHEEL_DIAMETER) / TICKS_PER_REV

PWM_END   = 999
PWM_STEP  = 50
MEASURE   = 4.0     # s per step (capture rise + steady state)
SAMPLE_DT = 0.2     # s between speed samples (the settling resolution)
SLIDE_WIN = 0.6     # s trailing window for the speed calc (smooths the ripple)
AVG_FROM  = 2.0     # s: average after this for the steady-state value
COOLDOWN  = 1.0     # s motor off between steps
# PWM values to draw on the settling plot (closest available are picked)
SETTLING_PWMS = [300, 500, 700, 999]

OUTPUT_DIR = os.path.expanduser('~/thesis_data/PID_tune_STM32')
os.makedirs(OUTPUT_DIR, exist_ok=True)
TIMESTAMP  = datetime.now().strftime('%Y%m%d_%H%M%S')
BASE = os.path.join(OUTPUT_DIR, f'RAW_openloop_elevated_{TIMESTAMP}')


def wrap16(d):
    d %= 65536
    if d >= 32768:
        d -= 65536
    return d


class TelemetryReader(threading.Thread):
    def __init__(self, ser):
        super().__init__(daemon=True)
        self.ser = ser
        self.lock = threading.Lock()
        self.total_l = 0
        self.total_r = 0
        self.prev_l = None
        self.prev_r = None
        self.packets = 0
        self.running = True

    def run(self):
        while self.running:
            try:
                line = self.ser.readline().decode('utf-8', errors='ignore').strip()
            except Exception:
                continue
            if not (line.startswith('{') and line.endswith('}')):
                continue
            try:
                pkt = json.loads(line)
            except json.JSONDecodeError:
                continue
            if 'l' not in pkt or 'r' not in pkt:
                continue
            with self.lock:
                if self.prev_l is None:
                    self.prev_l, self.prev_r = pkt['l'], pkt['r']
                else:
                    self.total_l += wrap16(pkt['l'] - self.prev_l)
                    self.total_r += wrap16(pkt['r'] - self.prev_r)
                    self.prev_l, self.prev_r = pkt['l'], pkt['r']
                self.packets += 1

    def snapshot(self):
        with self.lock:
            return self.total_l, self.total_r, self.packets

    def stop(self):
        self.running = False


def main():
    ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=0.2)
    time.sleep(0.5)
    reader = TelemetryReader(ser)
    reader.start()

    def send(p):
        ser.write(f"P:{p},{p}\r\n".encode('utf-8'))

    def stop():
        for _ in range(8):
            send(0)
            time.sleep(0.05)

    def to_cms(ticks, dt):
        return abs(ticks) / dt * M_PER_TICK * 100.0

    def measure(pwm):
        """Apply pwm, sample speed every SAMPLE_DT for MEASURE seconds. Speed is
        computed over a TRAILING window (SLIDE_WIN) of the cumulative tick total,
        so one occasionally-missed telemetry chunk does not cause a dip/spike
        (that was the 'ripple'). Returns (steady dict, series)."""
        _, _, p0 = reader.snapshot()
        t0 = time.time()
        snaps = [(t0, *reader.snapshot()[:2])]   # (t, total_l, total_r) history
        series = []
        next_s = t0 + SAMPLE_DT
        while time.time() < t0 + MEASURE:
            send(pwm)
            time.sleep(0.02)
            now = time.time()
            if now >= next_s:
                l, r, _ = reader.snapshot()
                snaps.append((now, l, r))
                # pick the oldest snapshot within the trailing window
                ref = snaps[0]
                for s in snaps:
                    if now - s[0] <= SLIDE_WIN:
                        ref = s
                        break
                dt = now - ref[0]
                if dt < 0.001:
                    dt = now - snaps[-2][0]
                    ref = snaps[-2]
                lc = to_cms(l - ref[1], dt)
                rc = to_cms(r - ref[2], dt)
                series.append((round(now - t0, 2), round(lc, 2), round(rc, 2)))
                next_s += SAMPLE_DT
                print(f"\r    {int(t0 + MEASURE - now):2d}s  "
                      f"L={lc:5.1f} R={rc:5.1f} cm/s ", end='', flush=True)
        print()
        _, _, p1 = reader.snapshot()
        ss = [(lc, rc) for (t, lc, rc) in series if t >= AVG_FROM] or \
             [(lc, rc) for (t, lc, rc) in series]
        # median, not mean, so an occasional sampling-glitch dip/spike is
        # ignored rather than pulling the steady-state value
        l_ss = statistics.median(c[0] for c in ss) if ss else 0.0
        r_ss = statistics.median(c[1] for c in ss) if ss else 0.0
        steady = {
            'pwm': pwm,
            'left_rpm':  round(l_ss / (M_PER_TICK * 100) / TICKS_PER_REV * 60, 1),
            'right_rpm': round(r_ss / (M_PER_TICK * 100) / TICKS_PER_REV * 60, 1),
            'left_cms':  round(l_ss, 2),
            'right_cms': round(r_ss, 2),
            'packets': p1 - p0,
        }
        return steady, series

    pwm_list = list(range(0, PWM_END, PWM_STEP))
    if PWM_END not in pwm_list:
        pwm_list.append(PWM_END)

    print("\n" + "=" * 64)
    print("  RAW OPEN-LOOP characteristic + settling (ELEVATED)")
    print("=" * 64)
    print(f"  PWM 0..{PWM_END} step {PWM_STEP}, {MEASURE:.0f}s/step, "
          f"sample {SAMPLE_DT:.1f}s.")
    print("  ELEVATE the robot, wheels free. Ctrl-C aborts and saves.")
    input("\n  Press ENTER to start...")
    time.sleep(0.5)
    _, _, pc = reader.snapshot()
    print(f"  Telemetry packets so far: {pc} "
          f"({'OK' if pc > 0 else 'NONE - check firmware/port!'})")

    steady_rows = []
    ts_rows = []     # (pwm, t, left_cms, right_cms)
    try:
        for idx, pwm in enumerate(pwm_list):
            print(f"\n  [{idx+1}/{len(pwm_list)}] PWM = {pwm}")
            steady, series = measure(pwm)
            stop()
            time.sleep(COOLDOWN)
            steady_rows.append(steady)
            for (t, lc, rc) in series:
                ts_rows.append({'pwm': pwm, 't_s': t, 'left_cms': lc, 'right_cms': rc})
            print(f"    steady: L={steady['left_cms']:.1f} "
                  f"R={steady['right_cms']:.1f} cm/s  ({steady['packets']} pkts)")
    except KeyboardInterrupt:
        print("\n  Aborted.")
    finally:
        stop()
        reader.stop()
        time.sleep(0.3)
        ser.close()

    if not steady_rows:
        print("  No data."); return

    # --- save steady-state characteristic ---
    with open(BASE + '.csv', 'w', newline='') as f:
        w = csv.DictWriter(f, fieldnames=[
            'pwm', 'left_rpm', 'right_rpm', 'left_cms', 'right_cms', 'packets'])
        w.writeheader()
        w.writerows(steady_rows)
    # --- save time series ---
    with open(BASE + '_timeseries.csv', 'w', newline='') as f:
        w = csv.DictWriter(f, fieldnames=['pwm', 't_s', 'left_cms', 'right_cms'])
        w.writeheader()
        w.writerows(ts_rows)

    print("\n" + "=" * 56)
    print(f"  {'PWM':>5} {'L rpm':>7} {'R rpm':>7} {'L cm/s':>7} {'R cm/s':>7}")
    print("  " + "-" * 40)
    for r in steady_rows:
        print(f"  {r['pwm']:>5} {r['left_rpm']:>7.1f} {r['right_rpm']:>7.1f} "
              f"{r['left_cms']:>7.2f} {r['right_cms']:>7.2f}")
    print(f"\n  Steady CSV:     {BASE}.csv")
    print(f"  Timeseries CSV: {BASE}_timeseries.csv")

    # --- plots ---
    try:
        import matplotlib
        matplotlib.use('Agg')
        import matplotlib.pyplot as plt

        # 1) steady-state characteristic
        pwms = [r['pwm'] for r in steady_rows]
        plt.figure(figsize=(8, 5))
        plt.plot(pwms, [r['left_cms'] for r in steady_rows], 'o-', label='Left motor')
        plt.plot(pwms, [r['right_cms'] for r in steady_rows], 's-', label='Right motor')
        plt.xlabel('PWM command (0-999)')
        plt.ylabel('Steady-state wheel speed (cm/s)')
        plt.title('Open-loop steady-state motor characteristic (elevated)')
        plt.grid(True, alpha=0.3); plt.legend()
        plt.savefig(BASE + '.png', dpi=150, bbox_inches='tight')
        print(f"  Characteristic plot: {BASE}.png")

        # 2) settling over time, for a few PWMs
        chosen = sorted({min(pwms, key=lambda p: abs(p - target))
                         for target in SETTLING_PWMS})

        def medfilt(v, w=5):
            out = []
            for i in range(len(v)):
                a = max(0, i - w // 2); b = min(len(v), i + w // 2 + 1)
                out.append(statistics.median(v[a:b]))
            return out

        plt.figure(figsize=(8, 5))
        for p in chosen:
            ts = sorted((r['t_s'], r['left_cms']) for r in ts_rows if r['pwm'] == p)
            if ts:
                plt.plot([t for t, _ in ts], medfilt([v for _, v in ts]), 'o-',
                         label=f'PWM {p} (left)')
        plt.xlabel('Time since PWM applied (s)')
        plt.ylabel('Wheel speed (cm/s)')
        plt.title('Motor settling to steady state (left wheel)')
        plt.grid(True, alpha=0.3); plt.legend()
        plt.savefig(BASE + '_settling.png', dpi=150, bbox_inches='tight')
        print(f"  Settling plot:       {BASE}_settling.png")
    except Exception as e:
        print(f"  (plot skipped: {e})")


if __name__ == '__main__':
    main()
