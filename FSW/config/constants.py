"""
ARDVARC Flight Software

Author: Timothy Behrer
email: timothy.behrer@colorado.edu

Description:
    constants used by fsw

Notes:
    - 
"""
import sys
import cv2
from cv2 import aruco
import numpy as np
import os
import argparse
import numpy.typing as npt
from dataclasses import dataclass
## TODO Check that all imports are correct


"""Pi Camera Parameters"""
##### Old Camera Parameters #####
"""FOCAL_LENGTH: float = 1.4 # mm
CAMERA_SENSOR_WIDTH: float = 3.6 # mm
CAMERA_SENSOR_HEIGHT: float = 2.4 # mm
FRAME_WIDTH: int = 640 # pixels
FRAME_HEIGHT: int = 480 # pixels
CAMERA_SIZE: list = [FRAME_WIDTH, FRAME_HEIGHT] # pixels x pixels
VIDEO_FPS: int = 60 # frames per second"""
##### Old Camera Parameters #####
## TODO Could be determined from the camera calibration function from cv2.aruco
## TODO This Needs to be updated to the true camera intrinsic parameters
INTRINSICS_PI_CAMERA: npt.NDArray = np.array(((933.15867, 0, 657.59), (0, 933.1586, 400.36993), (0, 0, 1)))
## TODO This Needs to be updated to the true camera distortion parameters
## TODO Could be determined from the camera calibration function from cv2.aruco
DISTORTION: npt.NDArray = np.array((0.0, 0.0, 0.0, 0.0))
## TODO Update the formatting of the camera extrinsic parameters
## TODO Configure a way to get the camera extrinsic parameters accurately
EXTRINSICS_PI_CAMERA_DCM: npt.NDArray = np.array(((0, 0, 0), (0, 0, 0),(0, 0, 0))) #rvec,tvec for Camera from to UAS Frame
EXTRINSICS_PI_CAMERA_TVEC: npt.NDArray = np.array((0, 0, 0)) #rvec,tvec for Camera from to UAS Frame

"""ArUco"""
# DICTIONARY: cv2.aruco.Dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_6X6_50)
#Define the dictionary of ArUco markers Possible
## TODO Comment out the ArUco dictionary that is not being used
## TODO Determine the best ArUco dictionary to use
ARUCO_DICT = {
	"DICT_4X4_50": cv2.aruco.DICT_4X4_50,
	"DICT_4X4_100": cv2.aruco.DICT_4X4_100,
	"DICT_4X4_250": cv2.aruco.DICT_4X4_250,
	"DICT_4X4_1000": cv2.aruco.DICT_4X4_1000,
	"DICT_5X5_50": cv2.aruco.DICT_5X5_50,
	"DICT_5X5_100": cv2.aruco.DICT_5X5_100,
	"DICT_5X5_250": cv2.aruco.DICT_5X5_250,
	"DICT_5X5_1000": cv2.aruco.DICT_5X5_1000,
	"DICT_6X6_50": cv2.aruco.DICT_6X6_50,
	"DICT_6X6_100": cv2.aruco.DICT_6X6_100,
	"DICT_6X6_250": cv2.aruco.DICT_6X6_250,
	"DICT_6X6_1000": cv2.aruco.DICT_6X6_1000,
	"DICT_7X7_50": cv2.aruco.DICT_7X7_50,
	"DICT_7X7_100": cv2.aruco.DICT_7X7_100,
	"DICT_7X7_250": cv2.aruco.DICT_7X7_250,
	"DICT_7X7_1000": cv2.aruco.DICT_7X7_1000,
	"DICT_ARUCO_ORIGINAL": cv2.aruco.DICT_ARUCO_ORIGINAL,
	"DICT_APRILTAG_16h5": cv2.aruco.DICT_APRILTAG_16h5,
	"DICT_APRILTAG_25h9": cv2.aruco.DICT_APRILTAG_25h9,
	"DICT_APRILTAG_36h10": cv2.aruco.DICT_APRILTAG_36h10,
	"DICT_APRILTAG_36h11": cv2.aruco.DICT_APRILTAG_36h11
}