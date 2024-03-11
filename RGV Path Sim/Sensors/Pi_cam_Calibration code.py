import numpy as np
import cv2
import cv2.aruco as aruco

# Define the dictionary
aruco_dict = aruco.Dictionary_get(aruco.DICT_6X6_250)
board = aruco.CharucoBoard_create(11, 8, .015, 0.011, aruco_dict)

# # Initialize the detector parameters using default values
# parameters =  aruco.DetectorParameters_create()


# Arrays to store object points and image points from all images
all_corners = []
all_ids = []
decimator = 0
images = ["img1.jpg", "img2.jpg", "img3.jpg", "img4.jpg", "img5.jpg", "img6.jpg", "img7.jpg", "img8.jpg", "img9.jpg", "img10.jpg"
          , "img11.jpg", "img12.jpg", "img13.jpg", "img14.jpg", "img15.jpg", "img16.jpg", "img17.jpg", "img18.jpg", "img19.jpg", "img20.jpg"]

# Loop over your images
for img in images:
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    corners, ids, _ = aruco.detectMarkers(gray, aruco_dict)

    if len(corners) > 0:
        _, charuco_corners, charuco_ids = aruco.interpolateCornersCharuco(corners, ids, gray, board)

        all_corners.append(charuco_corners)
        all_ids.append(charuco_ids)

# Calibrate the camera
camera_matrix = np.zeros((3, 3))
dist_coeffs = np.zeros((5, 1))
_, camera_matrix, dist_coeffs, _, _ = aruco.calibrateCameraCharuco(all_corners, all_ids, board, gray.shape, camera_matrix, dist_coeffs)

# Save the calibration data
np.savez('calibration_data.npz', camera_matrix=camera_matrix, dist_coeffs=dist_coeffs)