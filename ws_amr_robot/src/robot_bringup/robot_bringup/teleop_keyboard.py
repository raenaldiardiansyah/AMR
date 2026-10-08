#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import sys
import termios
import tty
import threading

# 2026-10-08 (robot besar):
# - Publish Twist, bukan TwistStamped. twist_mux di hardware.launch.py memakai
#   use_stamped=False, jadi /cmd_vel_teleop harus Twist (TwistStamped tidak nyambung).
# - Kecepatan awal pelan untuk uji pertama, bisa dinaikkan/diturunkan dengan tombol.
LINEAR_START  = 0.20   # m/s
ANGULAR_START = 0.50   # rad/s
LINEAR_STEP   = 0.05
ANGULAR_STEP  = 0.10
LINEAR_MAX    = 0.60
ANGULAR_MAX   = 1.20


class TeleopKeyboard(Node):
    def __init__(self):
        super().__init__('teleop_keyboard')

        # Publisher for velocity commands (input twist_mux, prioritas teleop)
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel_teleop', 10)

        # Velocity settings (m/s and rad/s)
        self.linear_speed = LINEAR_START
        self.angular_speed = ANGULAR_START

        self.current_v = 0.0
        self.current_w = 0.0
        self.running = True

        # Timer to publish commands at 20Hz
        self.timer = self.create_timer(0.05, self.publish_cmd_vel)

        self.get_logger().info('Teleop Ready! (ROS Topic Mode)')
        self.print_usage()

    def print_usage(self):
        print("\n" + "="*60)
        print(" "*15 + "🤖 ROBOT TELEOP CONTROL 🤖")
        print("="*60)
        print("\n  Controls:")
        print("    ↑ UP     : Forward")
        print("    ↓ DOWN   : Backward")
        print("    ← LEFT   : Turn Left")
        print("    → RIGHT  : Turn Right")
        print("    SPACE    : Stop")
        print("    W / X    : Linear speed  + / -")
        print("    E / C    : Angular speed + / -")
        print("    Q/Ctrl+C : Quit (robot stop)")
        print(f"\n  Start: V={self.linear_speed:.2f} m/s, W={self.angular_speed:.2f} rad/s"
              f"  (max {LINEAR_MAX:.2f} / {ANGULAR_MAX:.2f})")
        print("="*60)
        print("  Status: Waiting for input...\n")

    def make_msg(self, v, w):
        msg = Twist()
        msg.linear.x = float(v)
        msg.angular.z = float(w)
        return msg

    def publish_cmd_vel(self):
        """Publish Twist message at 20Hz"""
        if not self.running:
            return
        self.cmd_vel_pub.publish(self.make_msg(self.current_v, self.current_w))

    def publish_stop(self):
        """Kirim beberapa perintah 0 sebelum keluar supaya robot pasti berhenti."""
        for _ in range(5):
            self.cmd_vel_pub.publish(self.make_msg(0.0, 0.0))

    def update_display(self, status_msg):
        """Update status display"""
        print(f"\r  Status: {status_msg:<60}", end='', flush=True)

    def speed_status(self):
        return f"V={self.linear_speed:.2f} m/s, W={self.angular_speed:.2f} rad/s"

    def keyboard_loop(self):
        """Keyboard input thread"""
        fd = sys.stdin.fileno()
        old_settings = termios.tcgetattr(fd)

        try:
            tty.setraw(sys.stdin.fileno())

            while self.running:
                key = sys.stdin.read(1)

                if key == '\x1b':  # ESC sequence
                    key += sys.stdin.read(2)

                    if key == '\x1b[A':  # UP
                        self.current_v = self.linear_speed
                        self.current_w = 0.0
                        self.update_display(f"⬆️  FORWARD (V={self.current_v:.2f} m/s)")

                    elif key == '\x1b[B':  # DOWN
                        self.current_v = -self.linear_speed
                        self.current_w = 0.0
                        self.update_display(f"⬇️  BACKWARD (V={self.current_v:.2f} m/s)")

                    elif key == '\x1b[D':  # LEFT
                        self.current_v = 0.0
                        self.current_w = self.angular_speed
                        self.update_display(f"⬅️  TURN LEFT (W={self.current_w:.2f} rad/s)")

                    elif key == '\x1b[C':  # RIGHT
                        self.current_v = 0.0
                        self.current_w = -self.angular_speed
                        self.update_display(f"➡️  TURN RIGHT (W={self.current_w:.2f} rad/s)")

                elif key == ' ':
                    self.current_v = 0.0
                    self.current_w = 0.0
                    self.update_display("⏹️  STOPPED")

                elif key in ('w', 'W', 'x', 'X', 'e', 'E', 'c', 'C'):
                    k = key.lower()
                    if k == 'w':
                        self.linear_speed = min(LINEAR_MAX, self.linear_speed + LINEAR_STEP)
                    elif k == 'x':
                        self.linear_speed = max(LINEAR_STEP, self.linear_speed - LINEAR_STEP)
                    elif k == 'e':
                        self.angular_speed = min(ANGULAR_MAX, self.angular_speed + ANGULAR_STEP)
                    else:
                        self.angular_speed = max(ANGULAR_STEP, self.angular_speed - ANGULAR_STEP)
                    # Speed baru berlaku di tombol arah berikutnya; gerakan sekarang tidak diubah.
                    self.update_display(f"⚙️  {self.speed_status()}")

                elif key in ('q', 'Q', '\x03'):  # \x03 = Ctrl+C (raw mode tidak memicu SIGINT)
                    self.current_v = 0.0
                    self.current_w = 0.0
                    print("\r\n\n  Shutting down teleop...\r")
                    self.publish_stop()
                    self.running = False
                    break

        finally:
            termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)


def main(args=None):
    rclpy.init(args=args)
    node = TeleopKeyboard()

    # Start keyboard thread
    keyboard_thread = threading.Thread(target=node.keyboard_loop, daemon=False)
    keyboard_thread.start()

    # Spin ROS
    try:
        while node.running and rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.1)
    except KeyboardInterrupt:
        print("\n\n  Interrupted by user")
    finally:
        node.running = False
        node.publish_stop()
        keyboard_thread.join(timeout=1.0)
        node.destroy_node()
        rclpy.shutdown()
        print("  Teleop shutdown complete.\n")


if __name__ == '__main__':
    main()
