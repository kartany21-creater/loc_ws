import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped
import tf2_ros
import math
import csv
from datetime import datetime


class OdomPublisher(Node):
    def __init__(self):
        super().__init__('odom_node')

        # パラメータ取得（サンプル周期）
        self.declare_parameter('sample_time', 0.3)
        self.sample_time = self.get_parameter('sample_time').value

        # 初期位置・角度
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        self.z = 0.0

        # Publisher
        self.odom_pub = self.create_publisher(Odometry, 'odom', 10)

        # TFブロードキャスタ
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)

        # Subscriber
        self.sub_z = self.create_subscription(Float64, 'z_estimate', self.z_callback, 10)
        self.sub_theta = self.create_subscription(Float64, 'theta_estimate', self.theta_callback, 10)

        # タイマ
        self.timer = self.create_timer(self.sample_time, self.timer_callback)

        # CSVファイル初期化
        now = datetime.now().strftime("%Y%m%d_%H%M%S")
        self.csv_path = f"odom_log_{now}.csv"
        with open(self.csv_path, "w") as f:
            f.write("time,x[m],y[m],theta[deg]\n")

    def z_callback(self, msg):
        self.z = msg.data

    def theta_callback(self, msg):
        self.theta = msg.data

    def timer_callback(self):
        # ロボットZ方向の速度をYaw角で世界座標に変換
        dx = self.z * math.cos(self.theta)
        dy = self.z * math.sin(self.theta)

        # 現在位置を更新（積分）
        self.x += dx
        self.y += dy

        # オドメトリメッセージ作成
        odom_msg = Odometry()
        odom_msg.header.stamp = self.get_clock().now().to_msg()
        odom_msg.header.frame_id = 'odom'
        odom_msg.child_frame_id = 'base_link'
        odom_msg.pose.pose.position.x = self.x
        odom_msg.pose.pose.position.y = self.y
        odom_msg.pose.pose.position.z = 0.0
        odom_msg.pose.pose.orientation.z = math.sin(self.theta / 2.0)
        odom_msg.pose.pose.orientation.w = math.cos(self.theta / 2.0)
        self.odom_pub.publish(odom_msg)

        # TFブロードキャスト
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'odom'
        t.child_frame_id = 'base_link'
        t.transform.translation.x = self.x
        t.transform.translation.y = self.y
        t.transform.translation.z = 0.0
        t.transform.rotation.z = math.sin(self.theta / 2.0)
        t.transform.rotation.w = math.cos(self.theta / 2.0)
        self.tf_broadcaster.sendTransform(t)

        # ログ保存
        timestamp = self.get_clock().now().to_msg().sec + self.get_clock().now().to_msg().nanosec * 1e-9
        with open(self.csv_path, "a") as f:
            f.write(f"{timestamp:.3f},{self.x:.3f},{self.y:.3f},{math.degrees(self.theta):.2f}\n")


def main(args=None):
    rclpy.init(args=args)
    node = OdomPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

