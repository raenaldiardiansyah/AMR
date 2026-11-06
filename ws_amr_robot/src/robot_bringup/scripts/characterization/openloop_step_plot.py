#!/usr/bin/env python3
"""
openloop_full_step.py  (v2 - bug fix: timestamp dan dt calculation)
=============================================================
Merekam dan memplot respons kecepatan motor dari diam (PWM=0)
ke PWM target secara open loop tanpa PID.

Hasilnya adalah kurva eksponensial khas sistem orde-1:
  speed(t) = Vmax * (1 - e^(-t/tau))

FIX dari versi sebelumnya:
  - Timestamp disimpan dalam detik float (bukan ms yang terpotong)
  - dt dihitung dari perbedaan waktu antar paket yang konsisten
  - Filter outlier: sample dengan dt > 0.5 detik diabaikan

FIRMWARE: freertos_characterization.c harus di-flash ke STM32.
          Script ini akan menolak jika firmware PID terdeteksi.

CARA JALANKAN:
    ~/Autonomous-Mobile-Robot/ws_amr_robot/src/robot_bringup/scripts/tools/kill_robot.sh && sleep 3
    python3 ~/openloop_full_step.py
    python3 ~/openloop_full_step.py --pwm 999 --hold 30
    python3 ~/openloop_full_step.py --pwm 999 --hold 60 --wheel left
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
# KONFIGURASI
# ============================================================
SERIAL_PORT  = '/dev/serial/by-id/usb-FTDI_FT232R_USB_UART_A5069RR4-if00-port0'
BAUD_RATE    = 115200
MM_PER_TICK  = 0.19635      # mm per tick = pi * 150mm / 2400 ticks (robot besar)
WHEEL_DIAM   = 0.150        # meter
TIMESTAMP    = datetime.now().strftime('%Y%m%d_%H%M%S')

# Batas dt yang valid: paket STM32 datang setiap 50ms (20Hz)
# Kita toleransi hingga 200ms (4x period) untuk jaga-jaga
DT_MIN = 0.005   # detik - hindari divisi dengan dt sangat kecil
DT_MAX = 0.200   # detik - abaikan sample jika gap terlalu besar (packet loss)


# ============================================================
# SERIAL
# ============================================================

def connect(port, baud):
    print(f"Menghubungkan ke {port}...")
    ser = serial.Serial(port, baud, timeout=0.15)
    ser.reset_input_buffer()
    time.sleep(0.6)
    # Accept both PID firmware (has gz/ax fields) and characterization firmware.
    # Both support P: raw PWM command (g_raw_mode=1, bypasses PID).
    ser.reset_input_buffer()
    print(f"  OK. Terhubung.")
    return ser


def send_pwm(ser, l, r):
    cmd = f"P:{max(0,min(999,int(l)))},{max(0,min(999,int(r)))}\r\n"
    ser.write(cmd.encode())


def read_packets(ser):
    """
    Baca semua paket JSON lengkap dari buffer serial.
    Return list of (packet_dict, recv_time_seconds).
    recv_time diambil SEBELUM readline supaya akurat.
    """
    results = []
    while ser.in_waiting > 0:
        recv_time = time.time()      # catat waktu terima SEKARANG
        try:
            line = ser.readline().decode('utf-8', errors='ignore').strip()
            if line.startswith('{') and line.endswith('}'):
                p = json.loads(line)
                if 'l' in p and 'r' in p:
                    results.append((p, recv_time))
        except Exception:
            pass
    return results


def stop(ser, duration=0.5):
    t = time.time()
    while (time.time() - t) < duration:
        send_pwm(ser, 0, 0)
        time.sleep(0.05)


# ============================================================
# REKAM DATA
# ============================================================

def record(ser, pwm, wheel, hold_secs):
    """
    Rekam encoder feedback untuk satu sesi lengkap.

    Cara hitung speed yang benar:
    1. Setiap kali ada paket baru dari STM32, catat waktu penerimaan
       menggunakan time.time() (float detik, presisi microseconds)
    2. Hitung delta_ticks = ticks_sekarang - ticks_sebelumnya
    3. dt = recv_time_sekarang - recv_time_sebelumnya
    4. speed = delta_ticks * MM_PER_TICK / dt
    5. Abaikan sample jika dt < DT_MIN atau dt > DT_MAX

    Kolom t di CSV = elapsed detik sejak t=0 (float, bukan ms)
    """
    samples = []

    # State untuk perhitungan delta
    prev_pkt  = None
    prev_time = None   # float detik dari time.time()
    t_zero    = None   # waktu t=0 untuk elapsed
    # Accumulator for merging packets across timing hiccups
    accum_l   = 0
    accum_r   = 0
    accum_t   = 0.0    # accumulated wall-clock time

    def wrap16(d):
        d %= 65536
        if d >= 32768:
            d -= 65536
        return d

    # Number of packets accumulated in current window
    accum_n = 0

    def process(pkt, recv_time, phase, l_cmd, r_cmd):
        nonlocal prev_pkt, prev_time, t_zero
        nonlocal accum_l, accum_r, accum_t, accum_n

        if prev_pkt is None:
            prev_pkt  = pkt
            prev_time = recv_time
            return None

        py_dt = recv_time - prev_time

        if py_dt < DT_MIN:
            prev_pkt  = pkt
            prev_time = recv_time
            return None

        # Accumulate tick deltas
        accum_l += wrap16(pkt['l'] - prev_pkt['l'])
        accum_r += -wrap16(pkt['r'] - prev_pkt['r'])
        accum_t += py_dt
        accum_n += 1

        prev_pkt  = pkt
        prev_time = recv_time

        # Emit only when we have accumulated at least 2 normal packets
        # (~100ms). This merges both the 40/60ms Python timing jitter
        # AND the periodic I2C stalls (120-200ms gaps) with the
        # following packet, so the ticks and time always match.
        if accum_n < 2:
            return None

        l_mmps = (accum_l * MM_PER_TICK) / accum_t
        r_mmps = (accum_r * MM_PER_TICK) / accum_t

        def to_rpm(mmps):
            return (mmps / 1000.0) * 60.0 / (math.pi * WHEEL_DIAM)

        s = {
            't':          round(recv_time - t_zero, 4),
            'phase':      phase,
            'l_pwm':      l_cmd,
            'r_pwm':      r_cmd,
            'left_mmps':  round(l_mmps, 2),
            'right_mmps': round(r_mmps, 2),
            'left_rpm':   round(to_rpm(l_mmps), 2),
            'right_rpm':  round(to_rpm(r_mmps), 2),
            'l_ticks':    pkt['l'],
            'r_ticks':    pkt['r'],
            'dt':         round(accum_t, 5),
        }

        # Reset accumulator for next window
        accum_l = 0
        accum_r = 0
        accum_t = 0.0
        accum_n = 0
        return s

    # Tentukan PWM per roda
    if wheel == 'left':
        l_cmd, r_cmd = pwm, 0
    elif wheel == 'right':
        l_cmd, r_cmd = 0, pwm
    else:
        l_cmd, r_cmd = pwm, pwm

    # ---- PHASE 1: DIAM 2 DETIK ----
    print("  Diam 2 detik...", end='', flush=True)
    ser.reset_input_buffer()
    stop(ser, 0.3)
    ser.reset_input_buffer()

    t_zero = time.time()
    t_end  = t_zero + 2.0

    while time.time() < t_end:
        send_pwm(ser, 0, 0)
        for pkt, rt in read_packets(ser):
            if prev_pkt is None:
                prev_pkt  = pkt
                prev_time = rt
                continue
            s = process(pkt, rt, 'diam', 0, 0)
            if s:
                samples.append(s)
        time.sleep(0.02)
    print(f" OK  ({len([s for s in samples if s['phase']=='diam'])} sampel)")

    # ---- PHASE 2: STEP + HOLD ----
    print(f"  Step ke PWM {pwm}, hold {hold_secs}s...")

    t_step_start = time.time()
    t_end        = t_step_start + hold_secs
    last_print   = t_step_start

    while time.time() < t_end:
        send_pwm(ser, l_cmd, r_cmd)
        for pkt, rt in read_packets(ser):
            s = process(pkt, rt, 'step', l_cmd, r_cmd)
            if s:
                samples.append(s)

        now = time.time()
        if (now - last_print) >= 5.0:
            hold_s = [x for x in samples if x['phase'] == 'step']
            elapsed_h = now - t_step_start
            if hold_s:
                last = hold_s[-1]
                print(f"    t={elapsed_h:.0f}s/{hold_secs}s  "
                      f"L={last['left_mmps']:.0f}mm/s "
                      f"({last['left_rpm']:.1f}RPM)  "
                      f"R={last['right_mmps']:.0f}mm/s "
                      f"({last['right_rpm']:.1f}RPM)")
            last_print = now

        time.sleep(0.02)

    n_step = len([s for s in samples if s['phase'] == 'step'])
    print(f"  Step selesai. {n_step} sampel.")

    # ---- PHASE 3: COAST DOWN 3 DETIK ----
    print("  Stop. Coast down 3s...", end='', flush=True)
    for _ in range(10):
        send_pwm(ser, 0, 0)
        time.sleep(0.05)

    t_end = time.time() + 3.0
    while time.time() < t_end:
        send_pwm(ser, 0, 0)
        for pkt, rt in read_packets(ser):
            s = process(pkt, rt, 'coast', 0, 0)
            if s:
                samples.append(s)
        time.sleep(0.02)
    print(f" OK")

    print(f"  Total: {len(samples)} sampel valid dari semua phase.")
    return samples


# ============================================================
# ANALISIS
# ============================================================

def analyze(samples, pwm):
    step = [s for s in samples if s['phase'] == 'step']
    if len(step) < 10:
        return None

    # Steady state: 60% akhir dari phase step
    ss_idx = int(len(step) * 0.4)
    ss     = step[ss_idx:]

    l_vals = [abs(s['left_mmps'])  for s in ss]
    r_vals = [abs(s['right_mmps']) for s in ss]

    l_avg = sum(l_vals) / len(l_vals)
    r_avg = sum(r_vals) / len(r_vals)
    l_std = math.sqrt(sum((x-l_avg)**2 for x in l_vals) / len(l_vals))
    r_std = math.sqrt(sum((x-r_avg)**2 for x in r_vals) / len(r_vals))

    def to_rpm(mmps):
        return (mmps / 1000.0) * 60.0 / (math.pi * WHEEL_DIAM)

    # Rise time: pertama mencapai 90% dari SS speed
    t0    = step[0]['t']
    thr90 = l_avg * 0.9
    rise  = None
    for s in step:
        if abs(s['left_mmps']) >= thr90:
            rise = round(s['t'] - t0, 3)
            break

    # Fit eksponensial: v(t) = Vmax*(1 - exp(-t/tau))
    fit = _fit_exp([s['t'] - t0 for s in step],
                   [abs(s['left_mmps']) for s in step])

    return {
        'pwm':       pwm,
        'l_ss':      round(l_avg, 1),
        'r_ss':      round(r_avg, 1),
        'l_rpm':     round(to_rpm(l_avg), 1),
        'r_rpm':     round(to_rpm(r_avg), 1),
        'l_std':     round(l_std, 1),
        'r_std':     round(r_std, 1),
        'noise_pct': round(l_std / l_avg * 100, 1) if l_avg > 1 else 0,
        'sym_diff':  round(abs(l_avg - r_avg), 1),
        'rise_time': rise,
        'fit_Vmax':  round(fit[0], 1) if fit else None,
        'fit_tau':   round(fit[1], 3) if fit else None,
    }


def _fit_exp(times, speeds):
    """
    Fit: speed(t) = Vmax * (1 - exp(-t/tau))
    Linearisasi: ln(1 - speed/Vmax_guess) = -t/tau
    Return (Vmax, tau) atau None.
    """
    if len(times) < 5:
        return None
    Vmax_g = max(speeds) * 1.05
    if Vmax_g <= 0:
        return None

    vt, vy = [], []
    for t, s in zip(times, speeds):
        r = s / Vmax_g
        if 0.02 < r < 0.96:
            try:
                vy.append(math.log(1.0 - r))
                vt.append(t)
            except ValueError:
                pass

    if len(vt) < 4:
        return None

    n   = len(vt)
    st  = sum(vt)
    sy  = sum(vy)
    st2 = sum(x**2 for x in vt)
    sty = sum(x*y for x, y in zip(vt, vy))
    D   = n * st2 - st * st
    if abs(D) < 1e-12:
        return None

    a = (n * sty - st * sy) / D   # slope
    if a >= 0:
        return None

    return Vmax_g, -1.0 / a


def print_analysis(res):
    if not res:
        return
    print(f"\n{'='*55}")
    print(f"ANALISIS  PWM={res['pwm']}  ({res['pwm']/999*100:.0f}% duty cycle)")
    print(f"{'='*55}")
    if res['rise_time']:
        print(f"  Rise time (0 ke 90% SS) : {res['rise_time']:.3f} detik")
    else:
        print(f"  Rise time               : tidak terdeteksi")
    print(f"  Kiri  SS : {res['l_ss']:.1f} mm/s  =  {res['l_rpm']:.1f} RPM")
    print(f"  Kanan SS : {res['r_ss']:.1f} mm/s  =  {res['r_rpm']:.1f} RPM")
    print(f"  Noise    : {res['noise_pct']:.1f}%  (std={res['l_std']:.1f} mm/s)")
    print(f"  Simetri  : {res['sym_diff']:.1f} mm/s perbedaan L vs R")
    if res['fit_Vmax'] and res['fit_tau']:
        print(f"\n  Fit eksponensial (roda kiri):")
        print(f"    Vmax = {res['fit_Vmax']:.1f} mm/s")
        print(f"    tau  = {res['fit_tau']:.3f} detik")
        print(f"    v(t) = {res['fit_Vmax']:.1f} × (1 - e^(−t / {res['fit_tau']:.3f}))")


# ============================================================
# PLOT
# ============================================================

def plot(samples, res, pwm, wheel, outfile):
    try:
        import matplotlib
        matplotlib.use('Agg')
        import matplotlib.pyplot as plt
    except ImportError:
        print("matplotlib belum ada. Install: pip3 install matplotlib")
        return

    if not samples:
        return

    times   = [s['t']          for s in samples]
    l_spd   = [s['left_mmps']  for s in samples]
    r_spd   = [s['right_mmps'] for s in samples]
    l_rpm_v = [s['left_rpm']   for s in samples]
    r_rpm_v = [s['right_rpm']  for s in samples]
    phases  = [s['phase']      for s in samples]

    def moving_avg(data, window=7):
        out = []
        half = window // 2
        for i in range(len(data)):
            lo = max(0, i - half)
            hi = min(len(data), i + half + 1)
            out.append(sum(data[lo:hi]) / (hi - lo))
        return out

    l_spd_smooth   = moving_avg(l_spd)
    r_spd_smooth   = moving_avg(r_spd)
    l_rpm_smooth   = moving_avg(l_rpm_v)
    r_rpm_smooth   = moving_avg(r_rpm_v)

    # Batas phase
    pb = {}
    for t, ph in zip(times, phases):
        if ph not in pb:
            pb[ph] = [t, t]
        pb[ph][1] = t

    t0_step = pb['step'][0] if 'step' in pb else 0

    fig, axes = plt.subplots(2, 1, figsize=(14, 9), sharex=True)
    fig.suptitle(
        f'Open Loop Motor Step Response  |  PWM 0 → {pwm} '
        f'({pwm/999*100:.0f}% duty cycle)\n'
        f'Roda: {wheel}  |  Tanpa PID/feedback  |  '
        f'Firmware: Characterization',
        fontsize=12, fontweight='bold'
    )

    # Shading phase
    colors_phase = {
        'diam':  ('#BDBDBD', 0.30, 'Diam (PWM=0)'),
        'step':  ('#C8E6C9', 0.25, f'Step hold (PWM={pwm})'),
        'coast': ('#FFECB3', 0.30, 'Coast down'),
    }
    for ph, (col, alpha, lbl) in colors_phase.items():
        if ph in pb:
            for ax in axes:
                ax.axvspan(pb[ph][0], pb[ph][1], alpha=alpha,
                           color=col,
                           label=lbl if ax == axes[0] else '')

    # Panel 1: Speed mm/s
    ax1 = axes[0]
    if wheel != 'right':
        ax1.plot(times, l_spd, color='#1565C0', linewidth=0.5,
                 alpha=0.25)
        ax1.plot(times, l_spd_smooth, color='#1565C0', linewidth=2.0,
                 alpha=0.9, label='Roda kiri')
    if wheel != 'left':
        ax1.plot(times, r_spd, color='#C62828', linewidth=0.5,
                 alpha=0.25)
        ax1.plot(times, r_spd_smooth, color='#C62828', linewidth=2.0,
                 alpha=0.9, label='Roda kanan')

    # Kurva fit eksponensial
    if res and res['fit_Vmax'] and res['fit_tau'] and 'step' in pb:
        t0s  = pb['step'][0]
        t1s  = pb['step'][1]
        step_dur = t1s - t0s
        # Buat array waktu halus untuk kurva fit
        n_pts = max(200, int(step_dur / 0.01))
        t_fit = [t0s + i * (step_dur / n_pts) for i in range(n_pts + 1)]
        v_fit = [res['fit_Vmax'] *
                 (1 - math.exp(-(t - t0s) / res['fit_tau']))
                 for t in t_fit]
        ax1.plot(t_fit, v_fit, color='#E65100', linewidth=2.5,
                 linestyle='--',
                 label=(f"Fit: {res['fit_Vmax']:.0f}·(1−e^(−t/"
                        f"{res['fit_tau']:.2f}s))"))

    # Garis SS
    if res and 'step' in pb:
        ax1.hlines(res['l_ss'], pb['step'][0], pb['step'][1],
                   colors='#555555', linewidths=1.5, linestyles=':',
                   label=f"SS kiri = {res['l_ss']:.0f} mm/s")
        if wheel == 'both':
            ax1.hlines(res['r_ss'], pb['step'][0], pb['step'][1],
                       colors='#B71C1C', linewidths=1.5, linestyles=':',
                       label=f"SS kanan = {res['r_ss']:.0f} mm/s")

    ax1.axhline(y=0, color='black', linewidth=0.5, alpha=0.3)
    ax1.set_ylabel('Kecepatan (mm/s)', fontsize=11)
    ax1.set_title('Kecepatan Linear – Open Loop (tanpa PID)', fontsize=11)
    ax1.legend(loc='lower right', fontsize=9, ncol=2)
    ax1.grid(True, alpha=0.3)

    # Anotasi rise time
    if res and res['rise_time'] and 'step' in pb:
        t_rt = t0_step + res['rise_time']
        v_rt = res['l_ss'] * 0.9
        ax1.annotate(
            f"Rise time\n= {res['rise_time']:.2f}s",
            xy=(t_rt, v_rt),
            xytext=(t_rt + max(0.3, res['rise_time']), v_rt * 0.6),
            fontsize=9, color='navy',
            arrowprops=dict(arrowstyle='->', color='navy'),
            bbox=dict(boxstyle='round,pad=0.2',
                      facecolor='lightblue', alpha=0.8)
        )

    # Anotasi tau
    if res and res['fit_tau'] and 'step' in pb:
        t_tau = t0_step + res['fit_tau']
        v_tau = (res['fit_Vmax'] * (1 - math.exp(-1))
                 if res['fit_Vmax'] else 0)
        ax1.annotate(
            f"τ = {res['fit_tau']:.2f}s\n(63% Vmax)",
            xy=(t_tau, v_tau),
            xytext=(t_tau + max(0.3, res['fit_tau'] * 0.3),
                    v_tau * 1.4),
            fontsize=9, color='#BF360C',
            arrowprops=dict(arrowstyle='->', color='#BF360C'),
            bbox=dict(boxstyle='round,pad=0.2',
                      facecolor='#FFE0B2', alpha=0.8)
        )

    # Panel 2: RPM
    ax2 = axes[1]
    if wheel != 'right':
        ax2.plot(times, l_rpm_v, color='#1565C0', linewidth=0.5,
                 alpha=0.25)
        ax2.plot(times, l_rpm_smooth, color='#1565C0', linewidth=2.0,
                 alpha=0.9, label='Roda kiri (RPM)')
    if wheel != 'left':
        ax2.plot(times, r_rpm_v, color='#C62828', linewidth=0.5,
                 alpha=0.25)
        ax2.plot(times, r_rpm_smooth, color='#C62828', linewidth=2.0,
                 alpha=0.9, label='Roda kanan (RPM)')

    # Kurva fit RPM
    if res and res['fit_Vmax'] and res['fit_tau'] and 'step' in pb:
        rpm_Vmax = (res['fit_Vmax'] / 1000.0) * 60.0 / (math.pi * WHEEL_DIAM)
        t0s     = pb['step'][0]
        t1s     = pb['step'][1]
        step_dur = t1s - t0s
        n_pts   = max(200, int(step_dur / 0.01))
        t_fit   = [t0s + i * (step_dur / n_pts) for i in range(n_pts + 1)]
        r_fit   = [rpm_Vmax * (1 - math.exp(-(t - t0s) / res['fit_tau']))
                   for t in t_fit]
        ax2.plot(t_fit, r_fit, color='#E65100', linewidth=2.5,
                 linestyle='--',
                 label=f"Fit eksponensial (τ={res['fit_tau']:.2f}s)")

    if res and 'step' in pb:
        ax2.hlines(res['l_rpm'], pb['step'][0], pb['step'][1],
                   colors='#555555', linewidths=1.5, linestyles=':',
                   label=f"SS = {res['l_rpm']:.1f} RPM")

    ax2.axhline(y=0, color='black', linewidth=0.5, alpha=0.3)
    ax2.set_ylabel('RPM', fontsize=11)
    ax2.set_title('Kecepatan Rotasi (RPM) – Open Loop', fontsize=11)
    ax2.legend(loc='lower right', fontsize=9)
    ax2.grid(True, alpha=0.3)

    axes[-1].set_xlabel('Waktu (detik)', fontsize=11)

    plt.tight_layout()
    plt.savefig(outfile, dpi=150, bbox_inches='tight')
    plt.close()
    print(f"Plot disimpan: {outfile}")


# ============================================================
# SAVE CSV
# ============================================================

def save_csv(samples, filepath):
    if not samples:
        return
    fields = ['t', 'phase', 'l_pwm', 'r_pwm',
              'left_mmps', 'right_mmps',
              'left_rpm',  'right_rpm',
              'l_ticks', 'r_ticks', 'dt']
    with open(filepath, 'w', newline='') as f:
        import csv as cm
        w = cm.DictWriter(f, fieldnames=fields, extrasaction='ignore')
        w.writeheader()
        w.writerows(samples)
    print(f"CSV disimpan: {filepath}")


# ============================================================
# MAIN
# ============================================================

def main():
    parser = argparse.ArgumentParser(
        description='Open Loop Step Response (characterization firmware)')
    parser.add_argument('--pwm',   type=int, default=999,
                        help='Target PWM 0-999 (default: 999)')
    parser.add_argument('--hold',  type=int, default=15,
                        help='Hold time detik (default: 15)')
    parser.add_argument('--wheel', choices=['left','right','both'],
                        default='both')
    parser.add_argument('--outdir', default=os.path.expanduser('~/thesis_data/openloop_step_response'))
    args = parser.parse_args()

    pwm   = max(0, min(999, args.pwm))
    total = 2 + args.hold + 3

    print("=" * 55)
    print("OPEN LOOP FULL STEP RESPONSE  (v2)")
    print("=" * 55)
    print(f"PWM target   : {pwm}  ({pwm/999*100:.0f}% duty cycle)")
    print(f"Hold time    : {args.hold} detik")
    print(f"Roda         : {args.wheel}")
    print(f"Total waktu  : ~{total}s ({total/60:.1f} menit)")
    print()
    print("SYARAT: freertos_characterization.c di STM32")
    print("        ~/Autonomous-Mobile-Robot/ws_amr_robot/src/robot_bringup/scripts/tools/kill_robot.sh sudah dijalankan")
    print()
    input("Tekan ENTER untuk mulai...")

    ser = connect(SERIAL_PORT, BAUD_RATE)
    samples = []

    try:
        samples = record(ser, pwm, args.wheel, args.hold)
    except KeyboardInterrupt:
        print("\nDihentikan.")
    finally:
        for _ in range(10):
            send_pwm(ser, 0, 0)
            time.sleep(0.05)
        ser.close()
        print("Motor dihentikan. Serial ditutup.")

    if not samples:
        print("Tidak ada data.")
        return

    res = analyze(samples, pwm)
    print_analysis(res)

    outdir = os.path.expanduser(args.outdir)
    os.makedirs(outdir, exist_ok=True)

    csv_path = os.path.join(outdir, f'openloop_full_{TIMESTAMP}.csv')
    png_path = os.path.join(outdir, f'openloop_full_{TIMESTAMP}.png')

    save_csv(samples, csv_path)
    plot(samples, res, pwm, args.wheel, png_path)

    print(f"\nSelesai.")
    print(f"  CSV : {csv_path}")
    print(f"  PNG : {png_path}")


if __name__ == '__main__':
    main()
