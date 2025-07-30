import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped
import tf2_ros
import math
import csv
import os
from datetime import datetime

class OdomPublisher(Node):
    def __init__(self):
        super().__init__('odom_node')

        # === パラメータ ===
        self.declare_parameter('sample_time', 0.3)
        self.sample_time = self.get_parameter('sample_time').value

        # === 変数初期化 ===
        self.z_velocity = 0.0
        self.yaw = 0.0
        self.theta = 0.0  # yaw in radians

        self.x = 0.0
        self.y = 0.0
        self.elapsed_time = 0.0

        # === CSVログ出力用 ===
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        self.csv_path = f'odom_log_{timestamp}.csv'
        with open(self.csv_path, 'w', newline='') as f:
            writer = csv.writer(f)
            writer.writerow(['time', 'x', 'y', 'yaw_deg'])

        # === サブスクライバ ===
        self.subscription_vel = self.create_subscription(
            Float64,
            '/z_velocity',
            self.z_velocity_callback,
            10)

        self.subscription_yaw = self.create_subscription(
            Float64,
            '/yaw',
            self.yaw_callback,
            10)

        # === パブリッシャ・TF ===
        self.odom_publisher = self.create_publisher(Odometry, '/odom', 10)
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)

        # === タイマー起動 ===
        self.timer = self.create_timer(self.sample_time, self.timer_callback)

    def z_velocity_callback(self, msg):
        self.z_velocity = msg.data

    def yaw_callback(self, msg):
        self.yaw = msg.data
        self.theta = math.radians(self.yaw)

    def timer_callback(self):
        # === 自己位置更新 ===
        dx = self.z_velocity * self.sample_time * math.cos(self.theta)
        dy = self.z_velocity * self.sample_time * math.sin(self.theta)
        self.x += dx
        self.y += dy
        self.elapsed_time += self.sample_time

        # === Odometry メッセージ作成 ===
        odom_msg = Odometry()
        now = self.get_clock().now().to_msg()
        odom_msg.header.stamp = now
        odom_msg.header.frame_id = 'odom'
        odom_msg.child_frame_id = 'base_link'

        odom_msg.pose.pose.position.x = self.x
        odom_msg.pose.pose.position.y = self.y
        odom_msg.pose.pose.position.z = 0.0

        qz = math.sin(self.theta / 2.0)
        qw = math.cos(self.theta / 2.0)
        odom_msg.pose.pose.orientation.z = qz
        odom_msg.pose.pose.orientation.w = qw

        odom_msg.twist.twist.linear.x = self.z_velocity
        odom_msg.twist.twist.angular.z = 0.0

        self.odom_publisher.publish(odom_msg)

        # === TF 送信 ===
        t = TransformStamped()
        t.header.stamp = now
        t.header.frame_id = 'odom'
        t.child_frame_id = 'base_link'
        t.transform.translation.x = self.x
        t.transform.translation.y = self.y
        t.transform.translation.z = 0.0
        t.transform.rotation.z = qz
        t.transform.rotation.w = qw
        self.tf_broadcaster.sendTransform(t)

        # === CSVに記録 ===
        with open(self.csv_path, 'a', newline='') as f:
            writer = csv.writer(f)
            writer.writerow([round(self.elapsed_time, 3), round(self.x, 4), round(self.y, 4), round(self.yaw, 2)])

def main(args=None):
    rclpy.init(args=args)
    node = OdomPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

