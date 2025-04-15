# Import required modules 
import cv2 
import numpy as np 

# Cấu hình bàn cờ
CHECKERBOARD = (11, 12)  # Số giao điểm
square_size = 16.8   # Kích thước mỗi ô (mm)



# Danh sách lưu điểm
objpoints = []  # Điểm 3D thực tế
imgpoints = []  # Điểm 2D trong ảnh

# Tạo điểm 3D thực tế
objp = np.zeros((CHECKERBOARD[0] * CHECKERBOARD[1], 3), np.float32)
objp[:, :2] = np.mgrid[0:CHECKERBOARD[0], 0:CHECKERBOARD[1]].T.reshape(-1, 2)
objp *= square_size  # Chuyển sang đơn vị mm

# Mở camera
cap = cv2.VideoCapture(0)  # Thay đổi số nếu không nhận diện camera
if not cap.isOpened():
    print("❌ Không thể mở camera!")
    exit()

num_images = 0
print("Nhấn phím SPACE để chụp ảnh, ESC để thoát.")
while num_images < 40:  # Thu thập 20 ảnh có bàn cờ
    ret, frame = cap.read()
    if not ret:
        continue  # Bỏ qua frame lỗi

    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    found, corners = cv2.findChessboardCorners(gray, CHECKERBOARD, None)

    if found:
        criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)
        corners = cv2.cornerSubPix(gray, corners, (11, 11), (-1, -1), criteria)
        objpoints.append(objp)
        imgpoints.append(corners)
        cv2.drawChessboardCorners(frame, CHECKERBOARD, corners, found)
        num_images += 1
        print(f"✅ Ảnh {num_images}/40 được chụp")
    cv2.imshow('Camera Calibration', frame)

    if cv2.waitKey(1000) & 0xFF == 27:  # Nhấn SPACE để chụp ảnh nếu tìm thấy bàn cờ       
        break
    # elif key == 27:  # Nhấn SPACE để chụp ảnh nếu tìm thấy bàn cờ
    #     break

cap.release()
cv2.destroyAllWindows()

# Kiểm tra nếu không đủ ảnh
if len(objpoints) < 5:
    print("❌ Không đủ ảnh để hiệu chỉnh. Hãy thử lại!")
    exit()

# Tiến hành hiệu chỉnh camera
ret, camera_matrix, dist_coeffs,_,_ = cv2.calibrateCamera(objpoints, imgpoints, gray.shape[::-1], None, None)

# # Chuyển đổi từ pixel sang mm
# def pixel_to_mm(pixel_distance, square_size, imgpoints, CHECKERBOARD):
#     if len(imgpoints) > 0:
#         square_pixel_size = np.linalg.norm(imgpoints[0][CHECKERBOARD[0]] - imgpoints[0][0]) / (CHECKERBOARD[0] - 1)
#         real_distance = (pixel_distance * square_size) / square_pixel_size
#         return real_distance
#     return None

# # Ví dụ sử dụng: đo khoảng cách giữa hai điểm trong ảnh
# pixel_dist = np.linalg.norm(imgpoints[0][10] - imgpoints[0][0])  # Khoảng cách giả định giữa hai điểm trong ảnh (pixel)
# real_dist = pixel_to_mm(pixel_dist, square_size, imgpoints, CHECKERBOARD)
# print(f"📏 Khoảng cách thực tế: {real_dist:.2f} mm")

# In kết quả
print("📸 Camera Matrix:\n", camera_matrix)
print("🎯 Distortion Coefficients:\n", dist_coeffs)
# print("🔄 Rotation Vectors:\n", rvecs)
# print("🛑 Translation Vectors:\n", tvecs)

# Lưu tất cả thông số vào file .npz
np.savez("camera_params(2).npz", camera_matrix=camera_matrix, dist_coeffs=dist_coeffs, objpoints=objpoints, imgpoints=imgpoints, pixel_to_mm_ratio=square_size / np.linalg.norm(imgpoints[0][CHECKERBOARD[0]] - imgpoints[0][0]))
print("✅ Đã lưu tất cả thông số vào 'camera_params(2).npz'!")
