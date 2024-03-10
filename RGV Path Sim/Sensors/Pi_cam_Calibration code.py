import numpy as np
import cv2

import cv2.aruco as aruco

# Define the dictionary
aruco_dict = aruco.Dictionary_get(aruco.DICT_6X6_50)

# Initialize the detector parameters using default values
parameters =  aruco.DetectorParameters_create()


# Load the image
img = cv2.imread('image.png')

# Convert the image to gray scale
# gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

# Detect the markers in the image
corners, ids, rejectedImgPoints = aruco.detectMarkers(img, aruco_dict, parameters=parameters)

# If at least one marker detected
if len(corners) > 0:
    # Draw detected markers on the image
    aruco.drawDetectedMarkers(img, corners, ids)

    # Estimate pose of each marker and return the values rvec and tvec
    rvecs, tvecs, _ = aruco.estimatePoseSingleMarkers(corners, 0.05, camera_matrix, dist_coeffs)

    # # Draw axis for the markers
    # for i in range(len(rvecs)):
    #     aruco.drawAxis(img, camera_matrix, dist_coeffs, rvecs[i], tvecs[i], 0.1)



# Load the calibration data
calibration_data = np.load('calibration_data.npz')
camera_matrix = calibration_data['camera_matrix']
dist_coeffs = calibration_data['dist_coeffs']


# Display the resulting frame
cv2.imshow('frame', img)
cv2.waitKey(0)
cv2.destroyAllWindows()