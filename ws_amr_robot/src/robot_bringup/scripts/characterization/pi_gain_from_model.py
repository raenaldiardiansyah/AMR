#!/usr/bin/env python3
"""
pi_gain_from_model.py -- Hitung gain PI per roda dari model motor orde satu
---------------------------------------------------------------------------
Metode SAMA dengan Buku TA (Subbab 3.4.2-3.4.3):
  - Plant tiap motor: G(s) = K / (tau*s + 1)
      K   = kemiringan regresi PWM -> kecepatan tunak (mm/s per satuan PWM)
            -> dari open_loop_characterization.py
      tau = waktu ke 63,2% kecepatan tunak (detik)
            -> dari openloop_step_plot.py
  - Pole-zero cancellation: Ki = Kp / tau
  - Satu parameter bebas: tau_cl (konstanta waktu lingkar tertutup, TA = 0,30 s)
      Kp = tau / (K * tau_cl),  Ki = 1 / (K * tau_cl)
  - Cek: persamaan karakteristik tau*s^2 + (1 + K*Kp)*s + K*Ki = 0,
         Routh-Hurwitz (orde dua: stabil bila semua koefisien positif), zeta, akar.

Tidak bicara ke serial / ROS. Hanya menghitung.

USAGE:
  # rancang gain baru dari model motor robot besar
  python3 pi_gain_from_model.py --kl 0.207 --taul 0.397 --kr 0.239 --taur 0.394

  # cek gain yang sudah ada (mis. gain di firmware sekarang)
  python3 pi_gain_from_model.py --kl 0.207 --taul 0.397 --kr 0.239 --taur 0.394 \
      --cek 6.75 17.01 5.75 19.01
  -> harus sama dengan Buku TA: kiri 0,397 s^2 + 2,397 s + 3,521 = 0
"""
import argparse
import cmath


def design(K, tau, tau_cl):
    kp = tau / (K * tau_cl)
    ki = 1.0 / (K * tau_cl)
    return kp, ki


def analyze(name, K, tau, kp, ki):
    a2, a1, a0 = tau, 1.0 + K * kp, K * ki
    disc = a1 * a1 - 4 * a2 * a0
    r1 = (-a1 + cmath.sqrt(disc)) / (2 * a2)
    r2 = (-a1 - cmath.sqrt(disc)) / (2 * a2)
    wn = (a0 / a2) ** 0.5
    zeta = a1 / (2 * a2 * wn)
    stable = a2 > 0 and a1 > 0 and a0 > 0

    def fmt(r):
        return f"{r.real:.3f}" if abs(r.imag) < 1e-9 else f"{r.real:.3f}{r.imag:+.3f}j"

    print(f"\n  Motor {name}:  K = {K:.4f} (mm/s per PWM),  tau = {tau:.3f} s")
    print(f"    Kp = {kp:.2f}   Ki = {ki:.2f}   Ki/Kp = {ki / kp:.2f}   1/tau = {1 / tau:.2f}"
          f"   tau_cl = tau/(K*Kp) = {tau / (K * kp):.3f} s")
    print(f"    Karakteristik: {a2:.3f} s^2 + {a1:.3f} s + {a0:.3f} = 0")
    print(f"    Routh kolom 1: s^2 {a2:.3f} | s^1 {a1:.3f} | s^0 {a0:.3f}"
          f"  -> {'STABIL' if stable else 'TIDAK STABIL'}")
    kind = 'overdamped' if zeta > 1.0 + 1e-6 else ('kritis' if zeta > 1.0 - 1e-6 else 'underdamped')
    print(f"    Akar: {fmt(r1)}, {fmt(r2)}   zeta = {zeta:.2f} ({kind})")
    return stable


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--kl', type=float, required=True, help='K motor kiri (mm/s per PWM)')
    ap.add_argument('--taul', type=float, required=True, help='tau motor kiri (s)')
    ap.add_argument('--kr', type=float, required=True, help='K motor kanan (mm/s per PWM)')
    ap.add_argument('--taur', type=float, required=True, help='tau motor kanan (s)')
    ap.add_argument('--tau-cl', type=float, default=0.30, help='tau lingkar tertutup (default 0.30 s, sama dgn TA)')
    ap.add_argument('--cek', type=float, nargs=4, metavar=('KP_L', 'KI_L', 'KP_R', 'KI_R'),
                    help='analisis gain yang sudah ada, bukan merancang baru')
    a = ap.parse_args()

    if a.cek:
        kp_l, ki_l, kp_r, ki_r = a.cek
        print("=== CEK GAIN YANG ADA ===")
    else:
        kp_l, ki_l = design(a.kl, a.taul, a.tau_cl)
        kp_r, ki_r = design(a.kr, a.taur, a.tau_cl)
        print(f"=== RANCANG GAIN (pole-zero cancellation, tau_cl = {a.tau_cl:.2f} s) ===")

    ok = analyze('kiri', a.kl, a.taul, kp_l, ki_l)
    ok = analyze('kanan', a.kr, a.taur, kp_r, ki_r) and ok

    print("\n  Untuk firmware (AMR_STM32F407VET6/Core/Src/motor_control.c, Motor_Init):")
    print(f"    g_pid_left.kp  = {kp_l:.2f}f;")
    print(f"    g_pid_left.ki  = {ki_l:.2f}f;")
    print(f"    g_pid_right.kp = {kp_r:.2f}f;")
    print(f"    g_pid_right.ki = {ki_r:.2f}f;")
    if not ok:
        print("\n  PERINGATAN: ada sistem yang tidak stabil, jangan dipasang.")


if __name__ == '__main__':
    main()
