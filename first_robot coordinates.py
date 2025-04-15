import cv2  
import numpy as np
import collections
import time

# Load camera calibration parameters
calibration_data = np.load("camera_params(2).npz")
camera_matrix = calibration_data["camera_matrix"]  # Correct key name
distortion_coefficients = calibration_data["dist_coeffs"]  # Correct key name

# Reference: Chessboard square size in mm
SQUARE_SIZE_MM = 16.8
PATTERN_SIZE = (11, 12)  # Chessboard pattern (adjust as needed)
# Define real-world 3D coordinates of chessboard corners
obj_points = np.zeros((PATTERN_SIZE[0] * PATTERN_SIZE[1], 3), np.float32)
obj_points[:, :2] = np.mgrid[0:PATTERN_SIZE[0], 0:PATTERN_SIZE[1]].T.reshape(-1, 2) * SQUARE_SIZE_MM

# Open camera
cap = cv2.VideoCapture(0)

# Initialize a queue for smoothing camera position
num_samples = 10
position_history = collections.deque(maxlen=num_samples)

# Initialize Kalman Filter
kalman = cv2.KalmanFilter(6, 3)  # 6 state variables (x, y, z, vx, vy, vz), 3 measurements (x, y, z)
dt = 1  # Time step (can be adjusted)
kalman.measurementMatrix = np.eye(3, 6, dtype=np.float32)
kalman.transitionMatrix = np.array([[1, 0, 0, dt,  0,  0],  
                                    [0, 1, 0,  0, dt,  0],  
                                    [0, 0, 1,  0,  0, dt],  
                                    [0, 0, 0,  1,  0,  0],  
                                    [0, 0, 0,  0,  1,  0],  
                                    [0, 0, 0,  0,  0,  1]], dtype=np.float32)
kalman.processNoiseCov = np.eye(6, dtype=np.float32) * 1e-2  # Process noise
kalman.measurementNoiseCov = np.eye(3, dtype=np.float32) * 1e-1  # Measurement noise

def save_stable_position(position_history):
    if position_history:
        position_array = np.array(position_history)
        stable_position = np.median(position_array, axis=0)  # Using median as stable estimate
        
        try:
            data = np.load("camera_params(2).npz")
            saved_data = {key: data[key] for key in data.files}
        except FileNotFoundError:
            saved_data = {}
        
        saved_data["stable_position"] = stable_position
        np.savez("camera_params(2).npz", **saved_data)
        print("Stable position saved to camera_params(2).npz:", stable_position)

start_time = time.time()
while True:
    ret, frame = cap.read()
    if not ret:
        break
    
    # Undistort the image
    h, w = frame.shape[:2]
    new_camera_matrix, roi = cv2.getOptimalNewCameraMatrix(camera_matrix, distortion_coefficients, (w, h), 1, (w, h))
    undistorted = cv2.undistort(frame, camera_matrix, distortion_coefficients, None, new_camera_matrix)
    
    # Reduce noise
    blurred = cv2.GaussianBlur(undistorted, (5, 5), 0)
    gray = cv2.cvtColor(blurred, cv2.COLOR_BGR2GRAY)
    
    # Detect chessboard corners
    ret, corners = cv2.findChessboardCorners(gray, PATTERN_SIZE, None)
    if ret:
        # Refine corner accuracy
        corners = cv2.cornerSubPix(gray, corners, (11, 11), (-1, -1),
                                   criteria=(cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.01))
        
        # Solve for camera pose (rotation & translation relative to chessboard)
        _, rvec, tvec = cv2.solvePnP(obj_points, corners, camera_matrix, distortion_coefficients, flags=cv2.SOLVEPNP_ITERATIVE)
        
        # Convert rotation vector to rotation matrix
        R, _ = cv2.Rodrigues(rvec)
        
        # Compute camera position in robot coordinates
        camera_position = -R.T @ tvec
        position_history.append(camera_position.ravel())
        
        # Apply Kalman Filter
        measurement = np.array(camera_position.ravel(), dtype=np.float32).reshape(3, 1)
        kalman.correct(measurement)
        estimated_position = kalman.predict()[:3].ravel()
        
        print(f"Raw Position (robot coordinates): {camera_position.ravel()}")
        print(f"Filtered Position (robot coordinates): {estimated_position}")
        
        # Draw chessboard corners on the image
        cv2.drawChessboardCorners(undistorted, PATTERN_SIZE, corners, ret)
    
    # Show result
    cv2.imshow("Camera View", undistorted)
    cv2.waitKey(1)
    if time.time() - start_time > 10:
        break

save_stable_position(position_history)
cap.release()
cv2.destroyAllWindows()
