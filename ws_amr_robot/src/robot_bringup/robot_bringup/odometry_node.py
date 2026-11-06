#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32MultiArray
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Quaternion
import math

class OdometryNode(Node):
    def __init__(self):
        super().__init__('odometry_node')

        # --- KONFIGURASI FISIK ---
        self.wheel_diameter = 0.150   # ban 15 cm (lama: 0.068)
        self.wheel_base = 0.456       # jarak center ban kiri-kanan (lama: 0.299)
        self.ticks_per_rev = 2400.0   # 600 PPR x4, 1:1 (lama: 4557.0)

        # --- POLARITAS (Sesuai data empiris terakhir) ---
        self.polarity_left = 1.0
        self.polarity_right = -1.0

        # --- KONSTANTA ---
        self.m_per_tick = (math.pi * self.wheel_diameter) / self.ticks_per_rev

        # --- STATE POSISI ---
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0

        # --- VELOCITY STATE (NEW) ---
        # These were missing before. Without these, EKF gets zero velocity always.
        self.vx = 0.0
        self.vyaw = 0.0
        self.last_encoder_time = None  # Will be set on first encoder message

        self.prev_left_ticks = 0
        self.prev_right_ticks = 0
        self.initialized = False

        self.sub_enc = self.create_subscription(
            Int32MultiArray,
            'wheel_encoders',
            self.encoder_callback,
            10
        )

        self.pub_odom = self.create_publisher(Odometry, 'odom_raw', 10)

        self.create_timer(0.05, self.publish_odometry)

        self.get_logger().info('Odometry Node Started. Mode RAW with real velocity calculation.')

    def encoder_callback(self, msg):
        now = self.get_clock().now()

        current_left  = msg.data[0] * self.polarity_left
        current_right = msg.data[1] * self.polarity_right

        if not self.initialized:
            self.prev_left_ticks  = current_left
            self.prev_right_ticks = current_right
            self.last_encoder_time = now
            self.initialized = True
            return

        # --- CALCULATE TIME DELTA ---
        # Prefer the STM32-reported elapsed time (wheel_encoders[2], in ms)
        # over Pi wall clock. The STM32 dt is the exact window during which
        # those ticks were counted, so velocity = ticks / dt_stm32 is more
        # accurate than dividing by Pi scheduling time.
        # Fall back to Pi wall clock if old firmware (no index 2) or zero.
        dt_pi = (now - self.last_encoder_time).nanoseconds / 1e9
        self.last_encoder_time = now

        if len(msg.data) >= 3 and msg.data[2] > 0:
            dt = msg.data[2] / 1000.0
        else:
            dt = dt_pi

        if dt < 0.001:
            return

        # --- CALCULATE TICK DELTAS ---
        d_left_ticks  = current_left  - self.prev_left_ticks
        d_right_ticks = current_right - self.prev_right_ticks

        self.prev_left_ticks  = current_left
        self.prev_right_ticks = current_right

        # --- CONVERT TO METERS ---
        d_left  = d_left_ticks  * self.m_per_tick
        d_right = d_right_ticks * self.m_per_tick

        # --- UPDATE POSITION ---
        d_center = (d_left + d_right) / 2.0
        d_theta  = (d_right - d_left) / self.wheel_base

        self.x     += d_center * math.cos(self.theta)
        self.y     += d_center * math.sin(self.theta)
        self.theta += d_theta
        self.theta  = math.atan2(math.sin(self.theta), math.cos(self.theta))

        # --- CALCULATE REAL VELOCITY (NEW) ---
        # Divide distance traveled by time elapsed.
        # This gives actual speed in m/s and rad/s.
        # Before this fix, vx and vyaw were always 0.0 which confused the EKF.
        self.vx   = d_center / dt
        self.vyaw = d_theta  / dt

    def publish_odometry(self):
        now = self.get_clock().now()
        q = self.euler_to_quaternion(0, 0, self.theta)

        odom = Odometry()
        odom.header.stamp = now.to_msg()
        odom.header.frame_id = 'odom'
        odom.child_frame_id = 'base_link'

        # --- POSE ---
        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.orientation = q

        # Pose covariance.
        # X and Y are trusted (encoder is accurate over short distances).
        # Yaw reduced from 10.0 to 0.1 because PID now gives consistent motion.
        # We still do not fully trust encoder yaw because slip can happen.
        odom.pose.covariance[0]  = 0.05   # X uncertainty
        odom.pose.covariance[7]  = 0.05   # Y uncertainty
        odom.pose.covariance[35] = 0.1    # Yaw uncertainty (was 10.0, now reduced)

        # --- TWIST (VELOCITY) ---
        # These are now REAL values calculated from encoder deltas and time.
        # Before this fix these were hardcoded to 0.0 which broke the EKF.
        odom.twist.twist.linear.x  = self.vx
        odom.twist.twist.angular.z = self.vyaw

        # Twist covariance.
        # Velocity from encoder is fairly trustworthy on short timescales.
        odom.twist.covariance[0]  = 0.05   # Vx uncertainty
        odom.twist.covariance[35] = 0.05   # Vyaw uncertainty (was 10.0)

        self.pub_odom.publish(odom)

    def euler_to_quaternion(self, roll, pitch, yaw):
        qx = math.sin(roll/2) * math.cos(pitch/2) * math.cos(yaw/2) \
           - math.cos(roll/2) * math.sin(pitch/2) * math.sin(yaw/2)
        qy = math.cos(roll/2) * math.sin(pitch/2) * math.cos(yaw/2) \
           + math.sin(roll/2) * math.cos(pitch/2) * math.sin(yaw/2)
        qz = math.cos(roll/2) * math.cos(pitch/2) * math.sin(yaw/2) \
           - math.sin(roll/2) * math.sin(pitch/2) * math.cos(yaw/2)
        qw = math.cos(roll/2) * math.cos(pitch/2) * math.cos(yaw/2) \
           + math.sin(roll/2) * math.sin(pitch/2) * math.sin(yaw/2)
        return Quaternion(x=qx, y=qy, z=qz, w=qw)


def main(args=None):
    rclpy.init(args=args)
    node = OdometryNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
