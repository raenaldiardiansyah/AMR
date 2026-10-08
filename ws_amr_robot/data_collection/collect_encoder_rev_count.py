#!/usr/bin/env python3
"""
collect_encoder_rev_count.py -- Definitive encoder tick validation
------------------------------------------------------------------
Run one motor at a given PWM for a set time. You visually count how
many full wheel revolutions happen (put tape on the wheel, count passes).
The script counts encoder ticks and computes expected revolutions.

If encoder_revs matches visual_count: encoder is correct, tachometer was wrong.
If encoder_revs is short: encoder is losing ticks at high speed.

Also logs the firmware dt for every packet to verify timing.

USAGE:
  python3 collect_encoder_rev_count.py
"""

import serial
import json
import math
import time

SERIAL_PORT = '/dev/serial/by-id/usb-FTDI_FT232R_USB_UART_A5069RR4-if00-port0'
BAUD_RATE = 115200
TICKS_PER_REV = 2400.0


def wrap16(d):
    d %= 65536
    if d >= 32768:
        d -= 65536
    return d


def main():
    print("=" * 60)
    print(" ENCODER REVOLUTION COUNT TEST")
    print(" Put a tape mark on the wheel. Count full rotations visually.")
    print(" Compare vs encoder tick count.")
    print("=" * 60)

    ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=0.15)
    ser.reset_input_buffer()
    time.sleep(0.5)
    ser.reset_input_buffer()

    def send_pwm(l, r):
        ser.write(f"P:{int(l)},{int(r)}\r\n".encode())

    def stop():
        for _ in range(10):
            send_pwm(0, 0)
            time.sleep(0.05)

    def read_packets():
        pkts = []
        while ser.in_waiting > 0:
            try:
                line = ser.readline().decode('utf-8', errors='ignore').strip()
                if line.startswith('{') and line.endswith('}'):
                    p = json.loads(line)
                    if 'l' in p and 'r' in p:
                        pkts.append(p)
            except:
                pass
        return pkts

    stop()

    try:
        while True:
            pwm_str = input("\nPWM to test (0-999, or 'done'): ").strip()
            if pwm_str.lower() == 'done':
                break
            try:
                pwm = int(pwm_str)
            except ValueError:
                continue

            wheel = input("Which wheel? (left/right): ").strip().lower()
            if wheel not in ('left', 'right'):
                print("  Type 'left' or 'right'")
                continue

            duration = input("Run time in seconds (default 20): ").strip()
            duration = int(duration) if duration else 20

            if wheel == 'left':
                l_pwm, r_pwm = pwm, 0
            else:
                l_pwm, r_pwm = 0, pwm

            input(f"\n  Will run {wheel} motor at PWM {pwm} for {duration}s."
                  f"\n  Put tape mark on wheel. Ready to count."
                  f"\n  Press Enter to START...")

            # Flush and get first packet
            ser.reset_input_buffer()
            send_pwm(l_pwm, r_pwm)
            time.sleep(0.1)

            first_pkt = None
            while first_pkt is None:
                send_pwm(l_pwm, r_pwm)
                for p in read_packets():
                    first_pkt = p
                    break
                time.sleep(0.02)

            total_ticks = 0
            prev_pkt = first_pkt
            fw_dts = []
            pkt_count = 0

            t_start = time.time()
            t_end = t_start + duration

            while time.time() < t_end:
                send_pwm(l_pwm, r_pwm)
                time.sleep(0.02)
                for pkt in read_packets():
                    if wheel == 'left':
                        delta = wrap16(pkt['l'] - prev_pkt['l'])
                    else:
                        delta = -wrap16(pkt['r'] - prev_pkt['r'])
                    total_ticks += abs(delta)
                    prev_pkt = pkt
                    pkt_count += 1
                    if 'dt' in pkt:
                        fw_dts.append(int(pkt['dt']))

                remain = t_end - time.time()
                revs_so_far = total_ticks / TICKS_PER_REV
                print(f"\r  {remain:4.0f}s left | {total_ticks:7d} ticks | "
                      f"{revs_so_far:.2f} revs | {pkt_count} packets  ",
                      end='', flush=True)

            actual_elapsed = time.time() - t_start
            stop()
            print()

            encoder_revs = total_ticks / TICKS_PER_REV
            encoder_rpm = encoder_revs / (actual_elapsed / 60.0)

            print(f"\n  === RESULTS ({wheel} motor, PWM {pwm}) ===")
            print(f"  Elapsed:       {actual_elapsed:.2f}s")
            print(f"  Total ticks:   {total_ticks}")
            print(f"  Encoder revs:  {encoder_revs:.2f}")
            print(f"  Encoder RPM:   {encoder_rpm:.1f}")
            print(f"  Packets:       {pkt_count}")
            if fw_dts:
                avg_dt = sum(fw_dts) / len(fw_dts)
                print(f"  Firmware dt:   mean={avg_dt:.1f}ms  "
                      f"min={min(fw_dts)}  max={max(fw_dts)}")

            visual = input("\n  How many full rotations did you COUNT visually? "
                           "(or skip): ").strip()
            if visual:
                try:
                    v = float(visual)
                    visual_rpm = v / (actual_elapsed / 60.0)
                    err_revs = encoder_revs - v
                    err_pct = (err_revs / v * 100) if v > 0 else 0
                    print(f"\n  Visual count:  {v} revs = {visual_rpm:.1f} RPM")
                    print(f"  Encoder count: {encoder_revs:.2f} revs = {encoder_rpm:.1f} RPM")
                    print(f"  Difference:    {err_revs:+.2f} revs ({err_pct:+.1f}%)")
                    if abs(err_pct) < 3:
                        print(f"  >> ENCODER IS ACCURATE. Tachometer was likely wrong.")
                    else:
                        print(f"  >> {abs(err_pct):.0f}% discrepancy. Encoder may be "
                              f"{'losing' if err_pct < 0 else 'gaining'} ticks at this speed.")
                except ValueError:
                    pass

    except KeyboardInterrupt:
        print("\nInterrupted.")
    finally:
        stop()
        ser.close()
        print("Done.")


if __name__ == '__main__':
    main()
