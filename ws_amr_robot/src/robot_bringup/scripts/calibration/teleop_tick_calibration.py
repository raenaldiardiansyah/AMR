#!/usr/bin/env python3
"""
Kalibrasi tick encoder pakai TELEOP. Skrip hanya MEMBACA /wheel_encoders.
Gerakkan robot dari terminal teleop lain. Syarat: hardware.launch.py sudah jalan.

  roda   : python3 teleop_tick_calibration.py roda --putaran 5
           (roda terangkat, tandai ban, putar N putaran) -> ticks_per_rev
  tick   : python3 teleop_tick_calibration.py tick --jarak 2.0 --diameter 0.150
           (jalan lurus di lantai) -> ticks_per_rev
  lurus  : python3 teleop_tick_calibration.py lurus --jarak 2.0
           -> diameter efektif (asumsi 2400 tick)
  putar  : python3 teleop_tick_calibration.py putar --diameter 0.150 --putaran 1
           -> wheel_base efektif
Per percobaan: Enter di posisi awal -> gerakkan teleop -> STOP -> Enter. 'q' = selesai.
"""
import argparse
import math
import sys
import threading
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32MultiArray

TICKS_PER_REV = 2400.0
POLARITY_LEFT = 1.0     # samakan dengan odometry_node.py
POLARITY_RIGHT = -1.0


class Reader(Node):
    def __init__(self):
        super().__init__('teleop_tick_calibration')
        self.left = None
        self.right = None
        self.create_subscription(Int32MultiArray, '/wheel_encoders', self.cb, 10)

    def cb(self, msg):
        self.left = msg.data[0]
        self.right = msg.data[1]


def wait_enter(prompt):
    return input(prompt).strip().lower()


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('mode', choices=['lurus', 'tick', 'roda', 'putar'])
    ap.add_argument('--jarak', type=float, help='jarak terukur (m)')
    ap.add_argument('--diameter', type=float, help='diameter ban (m)')
    ap.add_argument('--putaran', type=float, default=1.0, help='jumlah putaran penuh')
    a = ap.parse_args()
    if a.mode == 'lurus' and not a.jarak:
        sys.exit('Mode lurus butuh --jarak (meter)')
    if a.mode == 'tick' and not (a.jarak and a.diameter):
        sys.exit('Mode tick butuh --jarak (m) dan --diameter (m)')
    if a.mode == 'putar' and not a.diameter:
        sys.exit('Mode putar butuh --diameter (meter)')

    rclpy.init()
    node = Reader()
    threading.Thread(target=rclpy.spin, args=(node,), daemon=True).start()

    print('Menunggu data /wheel_encoders ...')
    while node.left is None:
        time.sleep(0.05)
    print('Data masuk. Siap.\n')

    runs = []
    while True:
        cmd = wait_enter(f'[Percobaan {len(runs)+1}] Robot di posisi awal, tekan Enter (q = selesai): ')
        if cmd == 'q':
            break
        l0, r0 = node.left, node.right
        wait_enter('  Gerakkan dengan teleop, STOP di posisi akhir, tekan Enter: ')
        l1, r1 = node.left, node.right
        dl, dr = l1 - l0, r1 - r0
        print(f'  Delta tick: kiri={dl}  kanan={dr}')
        runs.append((dl, dr))

    if not runs:
        print('Belum ada data.')
        return

    if a.mode in ('tick', 'roda'):
        al = sum(abs(x[0]) for x in runs) / len(runs)
        ar = sum(abs(x[1]) for x in runs) / len(runs)
        if a.mode == 'tick':
            k = math.pi * a.diameter / a.jarak
        else:
            k = 1.0 / a.putaran
        tl, tr = al * k, ar * k
        avg = (tl + tr) / 2.0
        sel = abs(al - ar) / ((al + ar) / 2.0) * 100.0
        print('\n===== HASIL (TICK PER PUTARAN) =====')
        print(f'Percobaan       : {len(runs)}')
        print(f'Tick rata-rata  : kiri={al:.1f}  kanan={ar:.1f}  (selisih {sel:.2f}%)')
        print(f'ticks_per_rev   : kiri={tl:.1f}  kanan={tr:.1f}  rata-rata={avg:.1f}')
        print(f'Teori 600 PPR x4: {TICKS_PER_REV:.0f}  (beda {abs(avg-TICKS_PER_REV)/TICKS_PER_REV*100:.2f}%)')
        print('\nIsi ke:')
        print(f'  motor_control.c   TICKS_PER_REV   = {avg:.1f}f')
        print(f'  odometry_node.py  ticks_per_rev   = {avg:.1f}')
        if sel > 3:
            print('PERINGATAN: selisih kiri-kanan >3% -> cek slip/encoder/robot menyamping.')
    elif a.mode == 'lurus':
        al = sum(abs(x[0]) for x in runs) / len(runs)
        ar = sum(abs(x[1]) for x in runs) / len(runs)
        avg = (al + ar) / 2.0
        mm_per_tick = a.jarak * 1000.0 / avg
        diameter_mm = mm_per_tick * TICKS_PER_REV / math.pi
        sel = abs(al - ar) / avg * 100.0
        print('\n===== HASIL (LURUS) =====')
        print(f'Percobaan           : {len(runs)}')
        print(f'Rata-rata tick kiri : {al:.1f}   kanan: {ar:.1f}   selisih: {sel:.2f}%')
        print(f'MM_PER_TICK         : {mm_per_tick:.5f} mm')
        print(f'Diameter efektif    : {diameter_mm:.2f} mm  ({diameter_mm/1000:.4f} m)')
        print('\nIsi ke:')
        print(f'  motor_control.c     WHEEL_DIAMETER_MM = {diameter_mm:.2f}f')
        print(f'  odometry_node.py    wheel_diameter    = {diameter_mm/1000:.4f}')
        if sel > 3:
            print('PERINGATAN: selisih kiri-kanan >3% -> cek ban/encoder/robot menyamping.')
    else:
        m_per_tick = math.pi * a.diameter / TICKS_PER_REV
        vals = []
        for dl, dr in runs:
            d_l = dl * POLARITY_LEFT * m_per_tick
            d_r = dr * POLARITY_RIGHT * m_per_tick
            theta = a.putaran * 2.0 * math.pi
            vals.append(abs(d_r - d_l) / theta)
        wb = sum(vals) / len(vals)
        print('\n===== HASIL (PUTAR) =====')
        print(f'Percobaan         : {len(runs)}')
        print(f'wheel_base efektif: {wb:.4f} m  ({wb*1000:.1f} mm)')
        print('\nIsi ke:')
        print(f'  odometry_node.py  wheel_base     = {wb:.4f}')
        print(f'  freertos.c        WHEELBASE_MM   = {round(wb*1000)}')

    rclpy.shutdown()


if __name__ == '__main__':
    main()
