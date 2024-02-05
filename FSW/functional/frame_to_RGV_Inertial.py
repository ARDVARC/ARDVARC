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
import argparse
import imutils
import cv2
import sys

# Constants
"""Intrinsics"""
focal_length = 1.4 # mm
sensor_width = 3.6 # mm
sensor_height = 2.4 # mm
image_width = 640 # pixels
image_height = 480 # pixels
video_FPS = 30 # frames per second

class Camera:
    """Class for camera vison object recognition software"""

    def __init__(self, camera_id):
        """Initialize the camera"""
        self.camera = cv2.VideoCapture(camera_id)
        self.camera.set(cv2.CAP_PROP_FRAME_WIDTH, image_width)
        self.camera.set(cv2.CAP_PROP_FRAME_HEIGHT, image_height)
        self.camera.set(cv2.CAP_PROP_FPS, video_FPS)
        self.camera.set(cv2.CAP_PROP_OPENNI_FOCAL_LENGTH, focal_length)


    def frame_to_rgv_inert(frame):
    
    pass







