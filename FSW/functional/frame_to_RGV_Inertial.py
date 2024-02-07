"""
ARDVARC Flight Software

Author: Timothy Behrer
email: tibe0349@colorado.edu

Description:
    File for the camera vision and RGV inertial position estimate

Notes:
    TODO Implement camera vision object recognition software
    TODO Implement camera (in/ex)trinsics
    TODO Implement camera variance
    TODO Implement frame transition and UAS state to backout the RGV inertial position

    """
# Python 3.7
## Imports from the existing ArUco marker detection and annotation function
import sys
import cv2
from cv2 import aruco
import numpy as np
import os
import argparse
from dataclasses import dataclass
from config import constants
## TODO Make sure all imports are correct

## Function to detect ArUco markers
def detect_ArUco(camera):  
    parameters =  cv2.aruco.DetectorParameters()
    detector = cv2.aruco.ArucoDetector(constants.DICTIONARY, parameters)  
    # TODO I think there is a bug here in how camera is passed in
    ret, frame = camera.read()
    if ret == True:
        ## Get the ArUco directory
        (corners, ids, rejected) = detector.detectMarkers(frame)

        # verify *at least* one ArUco marker was detected
        if len(corners) > 0:
            # flatten the ArUco IDs list
            ids = ids.flatten()
            # loop over the detected ArUCo corners
            for (markerCorner, markerID) in zip(corners, ids):
                # extract the marker corners (which are always returned in
                # top-left, top-right, bottom-right, and bottom-left order)
                corners = markerCorner.reshape((4, 2))
                (topLeft, topRight, bottomRight, bottomLeft) = corners
                # convert each of the (x, y)-coordinate pairs to integers
                topRight = (int(topRight[0]), int(topRight[1]))
                bottomRight = (int(bottomRight[0]), int(bottomRight[1]))
                bottomLeft = (int(bottomLeft[0]), int(bottomLeft[1]))
                topLeft = (int(topLeft[0]), int(topLeft[1]))
                # draw the bounding box of the ArUCo detection in RED
                cv2.line(frame, topLeft, topRight, (0, 0, 255), 2)
                cv2.line(frame, topRight, bottomRight, (0, 0, 255), 2)
                cv2.line(frame, bottomRight, bottomLeft, (0, 0, 255), 2)
                cv2.line(frame, bottomLeft, topLeft, (0, 0, 255), 2)
                # compute and draw the center (x, y)-coordinates of the ArUco
                # marker
                cX = int((topLeft[0] + bottomRight[0]) / 2.0)
                cY = int((topLeft[1] + bottomRight[1]) / 2.0)
                cv2.circle(frame, (cX, cY), 4, (0, 0, 255), -1)
        


class Camera:
    """Class for camera vison object recognition software"""

    def __init__(self, camera_id):
        """Initialize the camera"""
        self.camera = cv2.VideoCapture(camera_id)
        self.camera.set(cv2.CAP_PROP_FRAME_WIDTH, constants.FRAME_WIDTH)
        self.camera.set(cv2.CAP_PROP_FRAME_HEIGHT,constants.FRAME_HEIGHT)
        self.camera.set(cv2.CAP_PROP_FPS, constants.VIDEO_FPS)
        self.camera.set(cv2.CAP_PROP_OPENNI_FOCAL_LENGTH, constants.FOCAL_LENGTH)
        if not self.camera.isOpened():
            print("Error opening video file")
            sys.exit()
        
    def frame_to_rgv_inert(frame):
        
        pass







if __name__ == "__main__":
    ## TODO Fix this for the unit test for detect_ArUco
    detect_ArUco(1,1)




######ARCHIVED CODE######
# ## dataclass for ArUco marker
# @dataclass
# class ArUcoMarker:

#     """ArUco marker"""
#     id: str
#     corners: np.ndarray
#     center: np.ndarray
#     pointing_vector: np.ndarray
#     recent_sighting: bool
#     corresponding_RGV: int
#     dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_6X6_50)
#     parameters =  cv2.aruco.DetectorParameters()
#     detector = cv2.aruco.ArucoDetector(dictionary, parameters)