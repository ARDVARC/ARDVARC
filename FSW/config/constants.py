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
from dataclasses import dataclass
## TODO Check that all imports are correct


"""Intrinsics"""
FOCAL_LENGTH: float = 1.4 # mm
CAMERA_SENSOR_WIDTH: float = 3.6 # mm
CAMERA_SENSOR_HEIGHT: float = 2.4 # mm
FRAME_WIDTH: int = 640 # pixels
FRAME_HEIGHT: int = 480 # pixels
CAMERA_SIZE: list = [FRAME_WIDTH, FRAME_HEIGHT] # pixels x pixels
VIDEO_FPS: int = 60 # frames per second


"""Extrinsics"""


"""ArUco"""
DICTIONARY: cv2.aruco.Dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_6X6_50)


"""Guidance"""
# TODO(LF) review this
# ask Rob what this should be for optimal bluetooth measurements 
ORBITAL_RADIUS_SINGLE = 10.0 # meters (ground distance)

# TODO(LF) review this
ORBITAL_RADIUS_JOINT = 10.0 # meters (ground distance)

# TODO(LF) review this
UAS_ALTITUDE_SETPOINT = 9.144 # meters (30 ft)
