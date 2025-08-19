import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64
import pyrealsense2 as rs
import numpy as np
import cv2
import time
import os
import json
from ultralytics import YOLO
from scipy.spatial.distance import cdist
from scipy.optimize import linear_sum_assignment
from sklearn.cluster import KMeans

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

class EstimateMultiNode(Node):
    def __init__(self):
        super().__init__('estimate_multi_node')

        self.model = YOLO('/home/maffin21/loc_ws/src/crop_estimation/yolo_model/best.pt')
        self.intr = load_intrinsics('/home/maffin21/loc_ws/src/crop_estimation/camera_intrinsics.json')

        self.pipeline = rs.pipeline()
        config = rs.config()
        config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
        config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
        self.pipeline.start(config)
        self.align = rs.align(rs.stream.color)

        self.trackers = {}
        self.next_id = 0

        self.z_pub = self.create_publisher(Float64, '/z_estimate', 10)
        self.vel_pub = self.create_publisher(Float64, '/z_velocity', 10)

        self.timer = self.create_timer(0.3, self.timer_callback)

    def timer_callback(self):
        frames = self.pipeline.wait_for_frames()
        aligned = self.align.process(frames)
        color_frame = aligned.get_color_frame()
        depth_frame = aligned.get_depth_frame()

        if not color_frame or not depth_frame:
            self.get_logger().warn("フレーム取得失敗")
            return

        color = np.asanyarray(color_frame.get_data())
        depth = np.asanyarray(depth_frame.get_data())

        results = self.model(color)[0]
        detections = []
        for box in results.boxes:
            x1, y1, x2, y2 = map(int, box.xyxy[0])
            z = get_bbox_depth_median(depth, x1, y1, x2, y2)
            if z == 0:
                continue
            cx, cy = (x1 + x2) // 2, y2
            xyz = deproject(cx, cy, z, self.intr)
            detections.append((cx, cy, xyz))

        predicted = []
        ids = list(self.trackers.keys())
        for tid in ids:
            self.trackers[tid][0].predict()
            predicted.append(self.trackers[tid][0].get_state())

        matched = set()
        matched_tid_map = {}
        if predicted and detections:
            detected_xyz = [d[2] for d in detections]
            dist_matrix = cdist(predicted, detected_xyz)
            row_ind, col_ind = linear_sum_assignment(dist_matrix)
            for r, c in zip(row_ind, col_ind):
                if dist_matrix[r][c] < 0.3:
                    tid = ids[r]
                    self.trackers[tid][0].update(detected_xyz[c])
                    self.trackers[tid][1] = 0
                    matched.add(c)
                    matched_tid_map[c] = tid

        for tid in ids:
            if tid not in matched_tid_map.values():
                self.trackers[tid][1] += 1

        to_delete = [tid for tid, (_, miss) in self.trackers.items() if miss > 1]
        for tid in to_delete:
            del self.trackers[tid]

        for i, (cx, cy, xyz) in enumerate(detections):
            if i not in matched:
                self.trackers[self.next_id] = [EKF3DTracker(xyz), 0]
                self.next_id += 1

        # --- 推定値のpublish（左右の平均） ---
        if len(self.trackers) >= 2:
            z_list = []
            dz_list = []

            # トラッカーのz値を元に左右クラスタリング（画面u座標で）
            tracker_items = [(tid, t[0].get_state(), t[0]) for tid, t in self.trackers.items()]
            u_coords = [int(t[1][0] * self.intr['fx'] / t[1][2] + self.intr['ppx']) for t in tracker_items]
            if len(u_coords) >= 2:
                kmeans = KMeans(n_clusters=2, n_init='auto').fit(np.array(u_coords).reshape(-1, 1))
                labels = kmeans.labels_

                left_zs, right_zs = [], []
                left_dz, right_dz = [], []

                for i, (tid, state, tracker) in enumerate(tracker_items):
                    z = state[2]
                    dz = tracker.get_z_movement(z)
                    if labels[i] == np.argmin(kmeans.cluster_centers_):
                        left_zs.append(z)
                        left_dz.append(dz)
                    else:
                        right_zs.append(z)
                        right_dz.append(dz)

                if left_zs and right_zs:
                    z_mean = (np.mean(left_zs) + np.mean(right_zs)) / 2
                    dz_mean = (np.mean(left_dz) + np.mean(right_dz)) / 2
                    speed = dz_mean / 0.3

                    msg_z = Float64()
                    msg_z.data = dz_mean
                    self.z_pub.publish(msg_z)

                    msg_v = Float64()
                    msg_v.data = speed
                    self.vel_pub.publish(msg_v)

def main(args=None):
    rclpy.init(args=args)
    node = EstimateMultiNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

