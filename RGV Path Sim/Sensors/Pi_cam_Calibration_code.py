import numpy as np
import cv2
import cv2.aruco as aruco


def calibrate_cam():
    # Define the dictionary
    aruco_dict = cv2.aruco.getPredefinedDictionary(aruco.DICT_6X6_250)
    board = cv2.aruco.CharucoBoard((11, 8), .015, .011, aruco_dict)
    params = cv2.aruco.DetectorParameters()
    detector = cv2.aruco.ArucoDetector(aruco_dict, params)
    charucodetector = cv2.aruco.CharucoDetector(board)



    # Arrays to store object points and image points from all images
    all_corners = []
    all_ids = []
    decimator = 0
    images = ["img1.jpg", "img2.jpg", "img3.jpg", "img4.jpg", "img5.jpg", "img6.jpg", "img7.jpg", "img8.jpg", "img9.jpg", "img10.jpg"
            , "img11.jpg", "img12.jpg", "img13.jpg", "img14.jpg"]


    # Loop over your images
    for img in images:
        image = cv2.imread(img)
        image_copy = image.copy()
        # cv2.imshow('CharUco Board', image_copy)
        # cv2.waitKey(0)
        

        (corners, ids, _) = detector.detectMarkers(image)
        # print(f"corners: {len(corners)}, ids: {ids}") ## Corners are being detected

        if len(corners) > 0:
            cv2.aruco.drawDetectedMarkers(image_copy, corners, ids)
            cv2.imshow('CharUco Board', image_copy)
            cv2.waitKey(0)
            ##All Ids are being detected

            image = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
            # cv2.imshow('CharUco Board', image)
            # cv2.waitKey(0)
            charuco_corners, charuco_ids, marker_corners, marker_ids = charucodetector.detectBoard(image) #This is where the detector is breaking down
            print(f"charuco_corners: {len(charuco_corners)}")
            all_corners.append(charuco_corners)
            all_ids.append(charuco_ids)
    # Calibrate the camera


    retval, camera_matrix, dist_coeffs, rvecs, tvecs = cv2.aruco.calibrateCameraAruco(corners=all_corners, ids=all_ids, board=board, imageSize=image.shape[:2], cameraMatrix=None, distCoeffs=None)
   
    # Save calibration data
    np.save('camera_matrix.npy', camera_matrix)
    np.save('dist_coeffs.npy', dist_coeffs)


    # Iterate through displaying all the images
    for image_file in images:
        image = cv2.imread(image_file)
        undistorted_image = cv2.undistort(image, camera_matrix, dist_coeffs)
        cv2.imshow('Undistorted Image', undistorted_image)
        cv2.waitKey(0)


calibrate_cam()

