import sys
import threading
import socket
import json
from PyQt5.QtWidgets import (
    QApplication, QWidget, QPushButton, QLabel, QTextEdit, QLineEdit,
    QFileDialog, QVBoxLayout, QHBoxLayout, QGroupBox, QGridLayout
)
from PyQt5.QtGui import QPixmap, QImage
from PyQt5.QtCore import Qt, pyqtSignal, QObject
import cv2
import numpy as np
import matplotlib.pyplot as plt
from weld_library import WeldDetectorPro

class ServerSignals(QObject):
    new_log = pyqtSignal(str)
    connection_status = pyqtSignal(bool)
    path_requested = pyqtSignal()

class WeldGUI(QWidget):
    def __init__(self):
        super().__init__()
        self.detector = None
        self.config_path = None
        self.path = []
        self.server_socket = None
        self.client_socket = None
        self.server_running = False
        self.server_thread = None
        self.signals = ServerSignals()
        self.initUI()
        self.setup_signal_connections()

    def setup_signal_connections(self):
        self.signals.new_log.connect(self.append_log)
        self.signals.connection_status.connect(self.update_connection_status)
        self.signals.path_requested.connect(self.handle_path_request)

    def initUI(self):
        self.setWindowTitle("Weld Detector Pro - Robot Server")
        self.setGeometry(100, 100, 900, 750)

        main_layout = QVBoxLayout()

        # --- Cấu hình & Server ---
        config_group = QGroupBox("🔧 Cấu hình và Kết nối Server")
        config_layout = QGridLayout()

        self.config_input = QLineEdit()
        browse_btn = QPushButton("Chọn config.yaml")
        browse_btn.clicked.connect(self.choose_config)

        self.load_btn = QPushButton("Tải cấu hình")
        self.load_btn.clicked.connect(self.load_detector)

        self.ip_input = QLineEdit("127.0.0.1")
        self.port_input = QLineEdit("5000")
        self.server_btn = QPushButton("Mở Server")
        self.server_btn.clicked.connect(self.toggle_server)

        config_layout.addWidget(QLabel("Đường dẫn config:"), 0, 0)
        config_layout.addWidget(self.config_input, 0, 1)
        config_layout.addWidget(browse_btn, 0, 2)

        config_layout.addWidget(QLabel("IP:"), 1, 0)
        config_layout.addWidget(self.ip_input, 1, 1)
        config_layout.addWidget(QLabel("Port:"), 2, 0)
        config_layout.addWidget(self.port_input, 2, 1)
        config_layout.addWidget(self.server_btn, 2, 2)

        config_layout.addWidget(self.load_btn, 3, 1)
        config_group.setLayout(config_layout)
        main_layout.addWidget(config_group)

        # --- Phát hiện ---
        action_group = QGroupBox("🤖 Phát hiện và Quản lý Đường hàn")
        action_layout = QHBoxLayout()

        self.detect_btn = QPushButton("Phát hiện mối hàn")
        self.detect_btn.clicked.connect(self.detect_weld)
        self.detect_btn.setEnabled(False)

        self.plot_btn = QPushButton("Xem đồ thị 3D")
        self.plot_btn.clicked.connect(self.show_plot)
        self.plot_btn.setEnabled(False)
        
        # Nút reset đường hàn hiện tại
        self.reset_path_btn = QPushButton("Reset đường hàn")
        self.reset_path_btn.clicked.connect(self.reset_current_path)
        self.reset_path_btn.setEnabled(False)

        action_layout.addWidget(self.detect_btn)
        action_layout.addWidget(self.plot_btn)
        action_layout.addWidget(self.reset_path_btn)
        action_group.setLayout(action_layout)
        main_layout.addWidget(action_group)

        # --- Hiển thị ảnh ROI ---
        self.image_label = QLabel("Ảnh camera sẽ hiển thị ở đây")
        self.image_label.setAlignment(Qt.AlignCenter)
        self.image_label.setFixedHeight(300)
        main_layout.addWidget(self.image_label)

        # --- Log ---
        self.connection_status_label = QLabel("Trạng thái kết nối: Chưa kết nối")
        self.status_label = QLabel("Trạng thái: Sẵn sàng")
        self.log_output = QTextEdit()
        self.log_output.setReadOnly(True)

        main_layout.addWidget(self.connection_status_label)
        main_layout.addWidget(self.status_label)
        main_layout.addWidget(self.log_output)

        self.setLayout(main_layout)

    def choose_config(self):
        path, _ = QFileDialog.getOpenFileName(self, 'Chọn file YAML', '', 'YAML Files (*.yaml)')
        if path:
            self.config_input.setText(path)
            self.config_path = path

    def load_detector(self):
        path = self.config_input.text()
        if not path:
            self.log_output.append("[ERROR] Bạn chưa chọn file config.")
            return
        try:
            self.detector = WeldDetectorPro(path)
            self.status_label.setText("Đã tải cấu hình thành công")
            self.log_output.append("[INFO] Cấu hình đã được tải.")
            self.detect_btn.setEnabled(True)
        except Exception as e:
            self.log_output.append(f"[ERROR] Không thể tải cấu hình: {e}")

    def toggle_server(self):
        if not self.server_running:
            self.start_server()
            self.server_btn.setText("Dừng Server")
        else:
            self.stop_server()
            self.server_btn.setText("Mở Server")

    def start_server(self):
        if self.server_running:
            return

        try:
            host = self.ip_input.text()
            port = int(self.port_input.text())
            
            self.server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self.server_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            self.server_socket.bind((host, port))
            self.server_socket.settimeout(1.0)  # Set timeout for socket operations
            
            self.server_running = True
            self.server_thread = threading.Thread(target=self.run_server, daemon=True)
            self.server_thread.start()
            
            self.log_output.append(f"[SERVER] Đang lắng nghe tại {host}:{port}")
        except Exception as e:
            self.log_output.append(f"[SERVER ERROR] Không thể khởi động server: {e}")

    def stop_server(self):
        if not self.server_running:
            return
            
        self.server_running = False
        if self.server_socket:
            try:
                self.server_socket.close()
            except:
                pass
        
        if self.client_socket:
            try:
                self.client_socket.close()
            except:
                pass
                
        self.client_socket = None
        self.log_output.append("[SERVER] Server đã dừng")
        self.signals.connection_status.emit(False)

    def run_server(self):
        try:
            self.server_socket.listen(1)
            self.signals.new_log.emit("[SERVER] Đang chờ robot kết nối...")
            
            while self.server_running:
                try:
                    self.server_socket.settimeout(1.0)
                    client, addr = self.server_socket.accept()
                    self.client_socket = client
                    self.signals.new_log.emit(f"[SERVER] Robot đã kết nối từ {addr}")
                    self.signals.connection_status.emit(True)
                    
                    # Sau khi kết nối, xử lý giao tiếp với robot
                    self.handle_robot_communication(client)
                except socket.timeout:
                    # Timeout là bình thường khi chờ kết nối
                    continue
                except Exception as e:
                    if self.server_running:  # Chỉ ghi log nếu server vẫn đang chạy
                        self.signals.new_log.emit(f"[SERVER ERROR] Lỗi chờ kết nối: {e}")
                    break
        finally:
            if self.server_socket:
                try:
                    self.server_socket.close()
                except:
                    pass
            self.signals.new_log.emit("[SERVER] Server đã dừng")

    def handle_robot_communication(self, client):
        while self.server_running:
            try:
                client.settimeout(1.0)
                data = client.recv(4096).decode().strip()
                
                if not data:  # Kết nối bị đóng
                    self.signals.new_log.emit("[SERVER] Robot đã ngắt kết nối")
                    self.signals.connection_status.emit(False)
                    break
                
                self.signals.new_log.emit(f"[ROBOT] {data}")
                
                if data == "READY_FOR_NEW_PATH":
                    self.signals.path_requested.emit()
                elif data.startswith("ERROR:"):
                    self.signals.new_log.emit(f"[ROBOT ERROR] {data}")
                
            except socket.timeout:
                # Timeout là bình thường khi chờ dữ liệu
                continue
            except Exception as e:
                if self.server_running:  # Chỉ ghi log nếu server vẫn đang chạy
                    self.signals.new_log.emit(f"[SERVER ERROR] Lỗi giao tiếp: {e}")
                break
                
        # Đóng kết nối khi thoát khỏi vòng lặp
        try:
            client.close()
        except:
            pass
        self.client_socket = None
        self.signals.connection_status.emit(False)

    def handle_path_request(self):
        if not hasattr(self, 'current_path_index'):
            self.current_path_index = 0
            
        if not self.path or len(self.path) == 0:
            self.signals.new_log.emit("[SERVER] Robot yêu cầu đường hàn, nhưng chưa có đường hàn nào")
            if self.client_socket:
                try:
                    self.client_socket.sendall(b"DONE")  # Gửi DONE khi không có đường hàn
                    self.signals.new_log.emit("[SERVER] Đã gửi DONE (không có đường hàn)")
                except:
                    self.signals.new_log.emit("[SERVER ERROR] Không thể gửi thông báo")
            return
            
        try:
            batch_size = 3  # Mỗi batch tối đa 4 điểm
            
            # Kiểm tra xem đã gửi hết điểm chưa
            if self.current_path_index >= len(self.path):
                # Đã gửi xong tất cả các điểm, gửi DONE
                if self.client_socket:
                    self.client_socket.sendall(b"DONE")
                    self.signals.new_log.emit("[SERVER] Đã gửi DONE (hoàn thành đường hàn)")
                    self.current_path_index = 0  # Reset lại để lần sau có thể gửi từ đầu
                return
            
            # Tính số điểm còn lại để gửi
            remaining_points = len(self.path) - self.current_path_index
            # Lấy số điểm cho batch hiện tại (tối đa batch_size)
            points_to_send = min(batch_size, remaining_points)
            
            # Lấy batch điểm tiếp theo
            current_batch = self.path[self.current_path_index:self.current_path_index + points_to_send]
            
            # Định dạng số thực với 2 chữ số thập phân để tăng độ chính xác
            path_str = ";".join([f"{point[0]:.2f},{point[1]:.2f},{point[2]:.2f}" for point in current_batch])
            
            if self.client_socket:
                self.client_socket.sendall(path_str.encode())
                self.signals.new_log.emit(f"[SERVER] Đã gửi batch {self.current_path_index//batch_size + 1}: {points_to_send} điểm (từ điểm {self.current_path_index + 1} đến {self.current_path_index + points_to_send})")
                
                # Cập nhật index cho batch tiếp theo
                self.current_path_index += points_to_send
                
        except Exception as e:
            self.signals.new_log.emit(f"[SERVER ERROR] Không thể gửi đường hàn: {e}")

    def detect_weld(self):
        self.status_label.setText("Đang phát hiện...")
        self.log_output.append("[INFO] Bắt đầu phát hiện...")
        self.detect_btn.setEnabled(False)

        def task():
            points, line, roi = self.detector.detect_weld()
            if points:
                self.status_label.setText("Phát hiện thành công")
                self.path = points
                self.line = line
                self.roi = roi
                self.display_image(roi)
                self.plot_btn.setEnabled(True)
                self.reset_path_btn.setEnabled(True)
                self.log_output.append("[SUCCESS] Mối hàn đã được phát hiện.")
                self.log_output.append(f"[INFO] Tạo đường hàn với {len(points)} điểm")
                self.log_output.append("[INFO] Robot có thể yêu cầu đường hàn này qua kết nối server")
                self.current_path_index = 0
            else:
                self.status_label.setText("Không phát hiện được")
                self.log_output.append("[WARNING] Không tìm thấy mối hàn.")
            self.detect_btn.setEnabled(True)

        threading.Thread(target=task).start()

    def display_image(self, img):
        rgb = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
        h, w, ch = rgb.shape
        bytes_per_line = ch * w
        qimg = QImage(rgb.data, w, h, bytes_per_line, QImage.Format_RGB888)
        pix = QPixmap.fromImage(qimg).scaled(800, 300, Qt.KeepAspectRatio)
        self.image_label.setPixmap(pix)

    def update_connection_status(self, connected):
        if connected:
            self.connection_status_label.setText("Trạng thái kết nối: Robot đã kết nối")
        else:
            self.connection_status_label.setText("Trạng thái kết nối: Robot không kết nối")

    def append_log(self, message):
        self.log_output.append(message)
        # Auto-scroll to bottom
        self.log_output.verticalScrollBar().setValue(
            self.log_output.verticalScrollBar().maximum()
        )

    def show_plot(self):
        try:
            X, Y, Z = zip(*self.path)
            lx = [self.line[0], self.line[2]]
            ly = [self.line[1], self.line[3]]
            lz = [Z[0], Z[-1]]
            fig = plt.figure()
            ax = fig.add_subplot(111, projection='3d')
            ax.plot(X, Y, Z, 'g-', label='Zigzag Path')
            ax.scatter(X, Y, Z, c='r', label='Waypoints')
            ax.plot(np.array(lx), np.array(ly), lz, 'b-', label='Detected Line')
            ax.set_xlabel("X (mm)")
            ax.set_ylabel("Y (mm)")
            ax.set_zlabel("Z (mm)")
            ax.legend()
            plt.show()
        except Exception as e:
            self.log_output.append(f"[ERROR] Không thể hiển thị đồ thị: {e}")

    def reset_current_path(self):
        """Reset đường hàn hiện tại và index để gửi lại từ đầu"""
        self.current_path_index = 0
        self.log_output.append("[INFO] Đã reset đường hàn - sẽ gửi lại từ đầu khi robot yêu cầu")
        self.reset_path_btn.setEnabled(len(self.path) > 0)

    def closeEvent(self, event):
        # Dừng server khi đóng ứng dụng
        self.stop_server()
        event.accept()

if __name__ == '__main__':
    app = QApplication(sys.argv)
    gui = WeldGUI()
    gui.show()
    sys.exit(app.exec_())