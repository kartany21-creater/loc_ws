import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
import serial
import time
import math

class UM7Node(Node):
    def __init__(self):
        super().__init__('um7_node')
        self.publisher_ = self.create_publisher(Float32, 'yaw', 10)
        self.timer = self.create_timer(0.1, self.timer_callback)

        self.port = '/dev/ttyUSB0'
        self.baudrate = 115200
        self.serial = None
        self.initial_yaw = None

        try:
            self.serial = serial.Serial(self.port, self.baudrate, timeout=1)
            self.get_logger().info("✅ UM7-LT 初期化成功")
        except serial.SerialException:
            self.get_logger().error(f"❌ シリアルポート {self.port} を開けません")
            exit(1)

    def timer_callback(self):
        try:
            line = self.serial.readline().decode('ascii', errors='ignore').strip()
            if line.startswith('$PCHRP'):
                parts = line.split(',')
                if len(parts) >= 8:
                    try:
                        yaw_deg = float(parts[7].split('*')[0])

                        if self.initial_yaw is None:
                            self.initial_yaw = yaw_deg

                        # オフセット補正
                        adj_yaw = yaw_deg - self.initial_yaw
                        # -180〜180°に正規化
                        adj_yaw = (adj_yaw + 180) % 360 - 180
                        # ラジアン変換
                        yaw_rad = math.radians(adj_yaw)

                        msg = Float32()
                        msg.data = yaw_rad
                        self.publisher_.publish(msg)

                        self.get_logger().info(f"📐 Yaw: {adj_yaw:.2f}° | {yaw_rad:.3f} rad")

                    except ValueError:
                        self.get_logger().warn(f"⚠ 数値変換エラー: {line}")
        except Exception as e:
            self.get_logger().error(f"❌ 読み取りエラー: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = UM7Node()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

