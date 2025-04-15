"""
WeldDetectorPro - Thu vien phat hien moi han va tao lo trinh cho robot ABB
"""

import cv2
import numpy as np
import socket
import yaml
import logging
import os
import matplotlib.pyplot as plt
from sklearn.cluster import DBSCAN
from datetime import datetime

class WeldDetectorPro:
    def __init__(self, config_path="config.yaml"):
        self._load_config(config_path)
        self._setup_logging()
        self._load_camera_params(self.cfg["camera_params_file"])

    def _load_config(self, path):
        with open(path, "r") as f:
            self.cfg = yaml.safe_load(f)

    def _setup_logging(self):
        os.makedirs("log", exist_ok=True)
        logging.basicConfig(
            filename="log/weld.log",
            level=logging.INFO,
            format="%(asctime)s [%(levelname)s] %(message)s",
        )
        self.logger = logging.getLogger("WeldDetectorPro")

    def _load_camera_params(self, file_path):
        data = np.load(file_path, allow_pickle=True)
        self.camera_matrix = data["camera_matrix"]
        self.dist_coeffs = data["dist_coeffs"]
        self.pixel_per_mm = data["pixel_to_mm_ratio"]
        self.stable_position = data.get("stable_position", [0, 0, 0])
        self.rotation_matrix = np.eye(3)
        self.translation_vector = np.array(self.stable_position)

    def detect_weld(self, camera_index=0):
        cap = cv2.VideoCapture(camera_index)
        frame_count = 0
        max_frames = self.cfg.get("max_frames", 100)

        while frame_count < max_frames:
            ret, frame = cap.read()
            if not ret:
                self.logger.warning("Khong doc duoc frame tu camera")
                break

            height, width = frame.shape[:2]
            roi = frame[int(height*self.cfg["roi_top"]):int(height*self.cfg["roi_bottom"]), int(width*self.cfg["roi_left"]):]
            edges = self._process_image(roi)

            lines = cv2.HoughLinesP(
                edges,
                1,
                np.pi/180,
                self.cfg["hough_thresh"],
                minLineLength=self.cfg["min_line_length"],
                maxLineGap=self.cfg["max_line_gap"]
            )

            if lines is not None:
                filtered = np.array([l[0] for l in lines])
                clustering = DBSCAN(
                    eps=self.cfg["dbscan_eps"],
                    min_samples=self.cfg["dbscan_min_samples"]
                ).fit(filtered)

                grouped_lines = [filtered[i] for i in range(len(filtered)) if clustering.labels_[i] != -1]
                center_x = roi.shape[1] / 2
                best_line, min_dist = None, float("inf")

                for line in grouped_lines:
                    mid_x = (line[0] + line[2]) / 2
                    if self._is_similar(line, grouped_lines) and abs(mid_x - center_x) < min_dist:
                        best_line, min_dist = line, abs(mid_x - center_x)

                if best_line is not None:
                    x1, y1, x2, y2 = best_line
                    cv2.line(roi, (x1, y1), (x2, y2), (0, 255, 0), 5)
                    points = self._generate_zigzag_path(best_line)
                    cap.release()
                    return points, best_line, roi

            frame_count += 1

        cap.release()
        self.logger.warning("Khong tim thay moi han trong gioi han frame")
        return None, None, None

    def _process_image(self, img):
        img = cv2.undistort(img, self.camera_matrix, self.dist_coeffs)
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        blur = cv2.GaussianBlur(gray, (5, 5), 0)
        canny = cv2.Canny(blur, self.cfg["canny_low"], self.cfg["canny_high"])
        kernel = np.ones((3, 3), np.uint8)
        return cv2.morphologyEx(canny, cv2.MORPH_CLOSE, kernel, iterations=2)

    def _is_similar(self, line, all_lines, tol=8):
        return sum(
            1 for other in all_lines
            if np.all(np.abs(np.array(line) - np.array(other)) < tol)
        ) >= 3

    def _generate_zigzag_path(self, line):
        x1, y1, x2, y2 = line
        step = self.cfg["zigzag_step"]
        amp = self.cfg["zigzag_amp"]
        z = self.cfg["zigzag_z"]

        length = np.hypot(x2 - x1, y2 - y1)
        num_pts = int(length / step)
        x_vals = np.linspace(x1, x2, num_pts)
        y_vals = np.linspace(y1, y2, num_pts)
        dx, dy = x2 - x1, y2 - y1
        norm = np.hypot(dx, dy)
        nx, ny = -dy / norm, dx / norm

        x_zigzag = x_vals + np.array([((-1)**i)*amp for i in range(num_pts)]) * nx
        y_zigzag = y_vals + np.array([((-1)**i)*amp for i in range(num_pts)]) * ny
        z_vals = np.full_like(x_vals, z)

        return list(zip(x_zigzag / self.pixel_per_mm, y_zigzag / self.pixel_per_mm, z_vals))

    def convert_to_robot_coords(self, points):
        if self.cfg["mode"] == "fixed":
            T = np.array(self.stable_position)
            return [tuple(np.array(p) + T) for p in points]
        else:
            R = self.rotation_matrix
            T = self.translation_vector
            return [tuple(R @ np.array(p) + T) for p in points]

    def send_to_robot(self, path, host="127.0.0.1", port=5000):
        """
        Gửi đường hàn đến robot theo định dạng phù hợp với mã RAPID
        Gửi theo từng batch nhỏ, mỗi batch tối đa 4 điểm
        """
        try:
            with socket.create_connection((host, port), timeout=30) as sock:
                self.logger.info(f"[Robot] Đã kết nối tới {host}:{port}")
                
                batch_size = 4  # Mỗi batch tối đa 4 điểm
                sent_count = 0
                
                while sent_count < len(path):
                    # Đợi tín hiệu READY_FOR_NEW_PATH từ robot
                    data = sock.recv(4096).decode().strip()
                    self.logger.info(f"[Robot] Nhận được tín hiệu: {data}")
                    
                    if data == "READY_FOR_NEW_PATH":
                        # Tính số điểm cho batch hiện tại
                        remaining = len(path) - sent_count
                        points_to_send = min(batch_size, remaining)
                        
                        if points_to_send > 0:
                            # Lấy batch điểm tiếp theo
                            current_batch = path[sent_count:sent_count + points_to_send]
                            
                            # Định dạng batch và gửi
                            batch_str = ";".join([f"{x:.2f},{y:.2f},{z:.2f}" for x, y, z in current_batch])
                            sock.sendall(batch_str.encode())
                            
                            self.logger.info(f"[Robot] Đã gửi batch {sent_count//batch_size + 1}: {points_to_send} điểm")
                            sent_count += points_to_send
                        else:
                            # Đã gửi hết tất cả điểm, gửi DONE
                            sock.sendall(b"DONE")
                            self.logger.info("[Robot] Đã gửi DONE (hoàn thành đường hàn)")
                            break
                    else:
                        self.logger.warning(f"[Robot] Nhận được tín hiệu không mong đợi: {data}")
                        return False
                
                return True
        except socket.error as e:
            self.logger.error(f"[Socket] Lỗi kết nối: {e}")
            return False

    def visualize(self, line, path, roi):
        os.makedirs("debug", exist_ok=True)
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        cv2.imwrite(f"debug/weld_roi_{timestamp}.jpg", roi)

        X, Y, Z = zip(*path)
        line_x = [line[0], line[2]]
        line_y = [line[1], line[3]]

        fig = plt.figure()
        ax = fig.add_subplot(111, projection='3d')
        ax.plot(X, Y, Z, 'g-', label='Zigzag Path')
        ax.scatter(X, Y, Z, c='r', label='Waypoints')
        ax.plot(np.array(line_x)/self.pixel_per_mm, np.array(line_y)/self.pixel_per_mm, [Z[0], Z[-1]], 'b-', label='Detected Line')
        ax.set_xlabel("X (mm)")
        ax.set_ylabel("Y (mm)")
        ax.set_zlabel("Z (mm)")
        ax.legend()
        plt.show()
