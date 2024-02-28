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
import rospy
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

"""State Machine Criteria"""
RECENT_ESTIMATE_TIME_CUTOFF: rospy.Duration = rospy.Duration.from_sec(2)
LOCALIZE_DURATION: rospy.Duration = rospy.Duration.from_sec(90)
JOINT_DURATION: rospy.Duration = rospy.Duration.from_sec(240)
RECENT_SIGHTING_TIME_CUTOFF: rospy.Duration = rospy.Duration.from_sec(2)
BATTERY_LOW_CHARGE_PCT_CUTOFF: float = 0.05 # Percent from 0 to 1
MINIMUM_LOCALIZE_DURATION: rospy.Duration = rospy.Duration.from_sec(60)
CONFIDENT_ESTIMATE_THRESHOLD: float = 0.2

"""Guidance"""
# TODO(LF) review this
# ask Rob what this should be for optimal bluetooth measurements 
# Aidan has some numbers that closer is better
ORBITAL_RADIUS_SINGLE = 1.0 # meters (ground distance)

# TODO(LF) review this
ORBITAL_RADIUS_JOINT = 10.0 # meters (ground distance)

# TODO(LF) review this
UAS_ALTITUDE_SETPOINT = 9.144 # meters (30 ft)

# Magic Number that's the center of the aerospace backyard in lat/long
# This is decimal lat/long, NOT mins, secs
AERO_BACKYARD_APPROX_CENTER = [40.010886, -105.243878]
AERO_BACKYARD_APPROX_ALT = 1614.001932 # meters