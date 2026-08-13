#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, TransformStamped
from nav_msgs.msg import Odometry
from tf2_ros import TransformBroadcaster
import serial
import math
import threading

WHEEL_BASE = 0.140          # ⚠️ Arduino code ထဲက WHEEL_BASE နဲ့ တူညီအောင် ပြင်ပါ
MAX_WHEEL_SPEED = 0.10     # m/s - 50RPM@6V motor အတွက် safety limit (~0.115 m/s max ရဲ့ ~85%)
SERIAL_PORT = '/dev/ttyACM0'   # ls /dev/tty* နဲ့ actual port စစ်ပါ
BAUD_RATE = 115200

class SerialBridge(Node):
    def __init__(self):
        super().__init__('cmd_vel_to_serial')

        self.ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=0.1)

        self.cmd_sub = self.create_subscription(
            Twist, '/cmd_vel', self.cmd_vel_callback, 10)

        self.odom_pub = self.create_publisher(Odometry, '/odom', 10)
        self.tf_broadcaster = TransformBroadcaster(self)

        self.read_thread = threading.Thread(target=self.read_serial_loop, daemon=True)
        self.read_thread.start()

        self.get_logger().info('cmd_vel -> serial bridge started.')

    def cmd_vel_callback(self, msg: Twist):
        linear_x = msg.linear.x
        angular_z = msg.angular.z

        vL = linear_x - (angular_z * WHEEL_BASE / 2.0)
        vR = linear_x + (angular_z * WHEEL_BASE / 2.0)

        # ⭐ Motor physical max speed ထက် မကျော်အောင် clamp
        vL = max(-MAX_WHEEL_SPEED, min(MAX_WHEEL_SPEED, vL))
        vR = max(-MAX_WHEEL_SPEED, min(MAX_WHEEL_SPEED, vR))

        cmd_str = f"{vL:.3f},{vR:.3f}\n"
        self.get_logger().info(f"Sending: {cmd_str.strip()}")
        try:
            self.ser.write(cmd_str.encode('utf-8'))
        except Exception as e:
            self.get_logger().warn(f"Serial write failed: {e}")

    def read_serial_loop(self):
        while rclpy.ok():
            try:
                line = self.ser.readline().decode('utf-8', errors='ignore').strip()
                if line:
                    self.get_logger().info(f"Received: {line}")  # ⭐ ဒီလိုင်း ထပ်ထည့်ပါ
                if line.startswith("ODOM,"):
                    parts = line.split(',')
                    if len(parts) == 6:
                        x = float(parts[1])
                        y = float(parts[2])
                        theta = float(parts[3])
                        vx = float(parts[4])
                        omega = float(parts[5])
                        self.publish_odom(x, y, theta, vx, omega)
            except Exception as e:
                self.get_logger().warn(f"Serial read error: {e}")

    def publish_odom(self, x, y, theta, vx, omega):
        now = self.get_clock().now().to_msg()

        qz = math.sin(theta / 2.0)
        qw = math.cos(theta / 2.0)

        odom = Odometry()
        odom.header.stamp = now
        odom.header.frame_id = 'odom'
        odom.child_frame_id = 'base_footprint'

        odom.pose.pose.position.x = x
        odom.pose.pose.position.y = y
        odom.pose.pose.orientation.z = qz
        odom.pose.pose.orientation.w = qw

        odom.twist.twist.linear.x = vx
        odom.twist.twist.angular.z = omega

        self.odom_pub.publish(odom)

        t = TransformStamped()
        t.header.stamp = now
        t.header.frame_id = 'odom'
        t.child_frame_id = 'base_footprint'
        t.transform.translation.x = x
        t.transform.translation.y = y
        t.transform.translation.z = 0.0
        t.transform.rotation.z = qz
        t.transform.rotation.w = qw

        self.tf_broadcaster.sendTransform(t)


def main(args=None):
    rclpy.init(args=args)
    node = SerialBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
