import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
import pyrealsense2 as rs
import numpy as np
import cv2
import time
import os
import json
from ultralytics import YOLO

def get_bbox_depth_median(depth, x1, y1, x2, y2):
    h, w = depth.shape
    x1, x2 = np.clip([x1, x2], 0, w)
    y1, y2 = np.clip([y1, y2], 0, h)
    patch = depth[y1:y2, x1:x2]
    valid_pixels = patch[patch > 0]
    if valid_pixels.size == 0:
        return 0.0
    return np.median(valid_pixels) * 0.001  # mm → m

def load_intrinsics(path="camera_intrinsics.json"):
    with open(path, 'r') as f:
        return json.load(f)

def deproject(u, v, z, intr):
    x = (u - intr['ppx']) * z / intr['fx']
    y = (v - intr['ppy']) * z / intr['fy']
    return [x, y, z]

class EKF3DTracker:
    def __init__(self, initial_pos):
        self.dt = 0.3
        self.state = np.hstack([initial_pos, np.zeros(3)])
        self.P = np.eye(6) * 0.01
        self.F = np.eye(6)
        for i in range(3):
            self.F[i, i+3] = self.dt
        self.Q = np.eye(6) * 0.001 * (self.dt ** 2)
        self.H = np.eye(3, 6)
        self.R = np.eye(3) * 0.01
        self.prev_z = initial_pos[2]

    def predict(self):
        self.state = self.F @ self.state
        self.P = self.F @ self.P @ self.F.T + self.Q

    def update(self, z):
        y = z - self.H @ self.state
        S = self.H @ self.P @ self.H.T + self.R
        K = self.P @ self.H.T @ np.linalg.inv(S)
        self.state = self.state + K @ y
        self.P = (np.eye(6) - K @ self.H) @ self.P

    def get_state(self):
        return self.state[:3]

    def get_z_movement(self, current_z):
        dz = current_z - self.prev_z
        self.prev_z = current_z
        return dz

class EstimateNode(Node):
    def __init__(self):
        super().__init__('estimate_node')
        self.publisher_ = self.create_publisher(Float32MultiArray, 'z_estimate', 10)
        self.model = YOLO('/home/maffin21/loc_ws/src/crop_estimation/yolo_model/best.pt')
        self.intr = load_intrinsics('/home/maffin21/loc_ws/src/crop_estimation/camera_intrinsics.json')

        self.pipeline = rs.pipeline()
        config = rs.config()
        config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
        config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
        self.pipeline.start(config)
        self.align = rs.align(rs.stream.color)

        self.timer = self.create_timer(0.3, self.timer_callback)
        self.tracker = None

        now = time.strftime("%Y%m%d_%H%M%S")
        self.csv_path = f"/home/maffin21/loc_ws/src/crop_estimation/output/trajectory_{now}.csv"
        os.makedirs(os.path.dirname(self.csv_path), exist_ok=True)
        with open(self.csv_path, "w") as f:
            f.write("time,z,speed\n")

    def timer_callback(self):
        frames = self.pipeline.wait_for_frames()
        aligned_frames = self.align.process(frames)
        color_frame = aligned_frames.get_color_frame()
        depth_frame = aligned_frames.get_depth_frame()

        if not color_frame or not depth_frame:
            self.get_logger().warn("❌ フレーム取得失敗")
            return

        color_image = np.asanyarray(color_frame.get_data())
        depth_image = np.asanyarray(depth_frame.get_data())

        # YOLO推論
        detections = []
        results = self.model(color_image)[0]
        for box in results.boxes:
            x1, y1, x2, y2 = map(int, box.xyxy[0])
            z = get_bbox_depth_median(depth_image, x1, y1, x2, y2)
            if z == 0: continue
            cx, cy = (x1 + x2) // 2, y2
            xyz = deproject(cx, cy, z, self.intr)
            detections.append(xyz)

        if not detections:
            self.get_logger().warn("⚠️ 株元が検出されませんでした")
            return

        target = min(detections, key=lambda p: p[2])  # 最も近い点を使用

        if self.tracker is None:
            self.tracker = EKF3DTracker(target)
            return

        self.tracker.predict()
        self.tracker.update(target)
        dz = self.tracker.get_z_movement(target[2])
        speed = dz / 0.3  # m/s

        current_z = self.tracker.get_state()[2]

        # Publish
        msg = Float32MultiArray()
        msg.data = [current_z, speed]
        self.publisher_.publish(msg)

        # Save to CSV
        timestamp = time.time()
        with open(self.csv_path, "a") as f:
            f.write(f"{timestamp:.3f},{current_z:.3f},{speed:.3f}\n")

def main(args=None):
    rclpy.init(args=args)
    node = EstimateNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

