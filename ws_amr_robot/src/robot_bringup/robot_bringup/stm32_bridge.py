#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import serial
import json
from std_msgs.msg import Int32MultiArray
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Imu


class STM32Bridge(Node):
    def __init__(self):
        super().__init__('stm32_bridge')

        # Serial connection
        self.port_name = '/dev/serial/by-id/usb-FTDI_FT232R_USB_UART_A5069RR4-if00-port0'
        self.baud_rate = 115200

        try:
            self.ser = serial.Serial(self.port_name, self.baud_rate, timeout=0.1)
            self.ser.reset_input_buffer()
            self.ser.reset_output_buffer()
            self.get_logger().info(f'Connected to STM32 at {self.port_name}')
        except serial.SerialException as e:
            self.get_logger().error(f'Failed to open serial port: {e}')
            exit(1)

        # Encoder tracking (overflow handling)
        self.prev_left_raw = 0
        self.prev_right_raw = 0
        self.total_left_ticks = 0
        self.total_right_ticks = 0
        self.is_first_message = True
        self.accumulated_dt_ms = 0  # sum of STM32 dt fields across all packets in one drain cycle

        # --- PUBLISHERS ---
        self.encoder_pub = self.create_publisher(Int32MultiArray, '/wheel_encoders', 10)
        self.imu_pub = self.create_publisher(Imu, '/imu/data_raw', 10)

        # --- SUBSCRIBERS ---
        self.cmd_vel_sub = self.create_subscription(
            Twist,
            '/cmd_vel',
            self.cmd_vel_callback,
            10
        )

        # Current velocity command
        self.current_v = 0  # mm/s
        self.current_w = 0  # mrad/s

        # 2026-10-08: safety timeout. twist_mux tidak mengirim 0 saat semua input
        # timeout, dan bridge mengulang perintah terakhir tiap 50 ms -> tanpa ini
        # robot terus jalan kalau teleop/Nav2 mati (timeout 2 s di firmware tidak
        # pernah terpicu karena perintah terus datang).
        self.cmd_timeout_s = 0.5
        self.last_cmd_time = self.get_clock().now()

        self.timer = self.create_timer(0.05, self.timer_callback)

        # Diagnostic counter (Session 5 addition): tracks how many JSON
        # messages were processed per cycle, for monitoring purposes only.
        # This does NOT affect behavior, just useful for verifying the fix.
        self.messages_per_cycle_log = []

        self.get_logger().info('STM32 Bridge Active (FIXED v2 - no dropped encoder messages)')

    def calculate_delta(self, current, previous):
        """Calculate delta with 16-bit overflow handling"""
        delta = current - previous
        if delta > 32768:
            delta -= 65536
        elif delta < -32768:
            delta += 65536
        return delta

    def cmd_vel_callback(self, msg):
        """Convert TwistStamped message to STM32 format"""
        self.current_v = int(msg.linear.x * 1000)   # m/s to mm/s
        self.current_w = int(msg.angular.z * 1000)  # rad/s to mrad/s

        # Clamp values
        self.current_v = max(-1000, min(1000, self.current_v))
        self.current_w = max(-2000, min(2000, self.current_w))
        self.last_cmd_time = self.get_clock().now()

    def process_telemetry_line(self, line, latest_imu_holder):
        try:
            data = json.loads(line)
        except json.JSONDecodeError:
            return  # skip malformed lines silently, same as original

        # --- A. ENCODER: accumulate delta immediately, don't lose data ---
        if 'l' in data and 'r' in data:
            raw_left = data['l']
            raw_right = data['r']

            if self.is_first_message:
                self.prev_left_raw = raw_left
                self.prev_right_raw = raw_right
                self.is_first_message = False
            else:
                delta_left = self.calculate_delta(raw_left, self.prev_left_raw)
                delta_right = self.calculate_delta(raw_right, self.prev_right_raw)

                self.total_left_ticks += delta_left
                self.total_right_ticks += delta_right

                self.prev_left_raw = raw_left
                self.prev_right_raw = raw_right

            # Accumulate the STM32-reported elapsed time for this packet.
            # We sum across all packets in one drain cycle so dt_ms matches
            # the total tick window being published, not just the last packet.
            if 'dt' in data:
                self.accumulated_dt_ms += int(data['dt'])

        # --- B. IMU: remember this reading, will use the LAST one seen ---
        if 'gz' in data:
            latest_imu_holder[0] = data

    def timer_callback(self):
        """Combined write command + read encoder + IMU data (FIXED)"""

        # 1. Send command to STM32 (stop if no fresh /cmd_vel within cmd_timeout_s)
        age_s = (self.get_clock().now() - self.last_cmd_time).nanoseconds / 1e9
        if age_s > self.cmd_timeout_s and (self.current_v != 0 or self.current_w != 0):
            self.get_logger().warn(f'/cmd_vel timeout ({age_s:.1f} s) -> STOP')
            self.current_v = 0
            self.current_w = 0
        try:
            cmd_str = f"V:{self.current_v},W:{self.current_w}\r\n"
            self.ser.write(cmd_str.encode('utf-8'))
        except Exception as e:
            self.get_logger().error(f'Send error: {e}')
            return

        # 2. Read ALL available data from STM32, processing EVERY valid
        #    line immediately so no encoder deltas are lost. Only the
        #    LAST IMU reading in this batch is kept (IMU is instantaneous,
        #    doesn't need accumulation).
        try:
            latest_imu = [None]  # mutable holder, see process_telemetry_line
            lines_processed = 0

            while self.ser.in_waiting > 0:
                line = self.ser.readline().decode('utf-8', errors='ignore').strip()
                if line.startswith('{') and line.endswith('}'):
                    self.process_telemetry_line(line, latest_imu)
                    lines_processed += 1

            # Diagnostic logging (optional, doesn't affect behavior)
            self.messages_per_cycle_log.append(lines_processed)
            if len(self.messages_per_cycle_log) > 1000:
                self.messages_per_cycle_log.pop(0)

            # --- Publish encoder totals ONCE per cycle, after processing
            #     ALL lines found, so nothing is lost ---
            if lines_processed > 0:
                msg_enc = Int32MultiArray()
                dt_to_pub = self.accumulated_dt_ms if self.accumulated_dt_ms > 0 else 50
                msg_enc.data = [self.total_left_ticks, self.total_right_ticks, dt_to_pub]
                self.encoder_pub.publish(msg_enc)
            self.accumulated_dt_ms = 0

            # --- Publish IMU using the LAST reading seen this cycle ---
            if latest_imu[0] is not None:
                data = latest_imu[0]
                imu_msg = Imu()
                imu_msg.header.stamp = self.get_clock().now().to_msg()
                imu_msg.header.frame_id = 'base_link'

                imu_msg.angular_velocity.z = float(data['gz']) / 1000.0
                imu_msg.linear_acceleration.x = float(data.get('ax', 0)) / 100.0
                imu_msg.linear_acceleration.y = float(data.get('ay', 0)) / 100.0
                imu_msg.linear_acceleration.z = float(data.get('az', 980)) / 100.0

                imu_msg.angular_velocity_covariance[8] = 0.005
                imu_msg.linear_acceleration_covariance[0] = 0.05
                imu_msg.linear_acceleration_covariance[4] = 0.05
                imu_msg.linear_acceleration_covariance[8] = 0.05

                imu_msg.orientation_covariance[0] = -1.0

                self.imu_pub.publish(imu_msg)

        except Exception as e:
            self.get_logger().error(f'Read error: {e}')

    def print_diagnostic_summary(self):
        """Call this manually (e.g. via a debug topic or shutdown hook)
        to see the distribution of messages-per-cycle, useful for
        confirming the fix is working as expected."""
        if not self.messages_per_cycle_log:
            return
        from collections import Counter
        counts = Counter(self.messages_per_cycle_log)
        self.get_logger().info(f'Messages-per-cycle distribution: {dict(counts)}')


def main(args=None):
    rclpy.init(args=args)
    node = STM32Bridge()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.print_diagnostic_summary()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()