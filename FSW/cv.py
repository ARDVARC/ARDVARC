""" 

ARDVARC Flight Software

Author: Tim Behrer
email: Timothy.Behrer@colorado.edu

Description:This is the CV node, it takes in the camera feed from ROS and annotates it for analysis, 
then publishes the annotated feed to the main state machine. It also performs calculations to 
determine the angular position of the ArUco markers. It will then publish the pointing vector 
to an appropriate ROS topic along with a recent sightings boolean to indicate if the marker has
been seen recently.

## Notes
- This node will be executed in parrallel with the main state machine.

## TODO create the ArUco marker detection and annotation function
## TODO create the pointing vector calculation function
## TODO create the recent sightings boolean function
## TODO Clean up the imports and improve the ArUco marker detection and annotation function
## TODO Ensure that coding is sufficiently modular and split into functions
"""




## Imports
import time
import rospy
import genpy
from rosardvarc.msg import AnnotatedCameraFrame, RecentSighting
from std_msgs.msg import Header, Time
from sensor_msgs.msg import Image
from functional.process_frame import detect_ArUco
## TODO Import the necessary message types.
"""from rosardvarc.msg import RawCameraFrame""" ## This is the subscriber to the camera frame.
"""from rosardvarc.msg import RecentSighting""" ## This is the publisher for the recent sightings.
"""from rosardvarc.msg import Uas2RgvPointingVector""" ## This is the publisher for the pointing vector.
import math
## Imports from the existing ArUco marker detection and annotation function
import sys
import cv2
from cv2 import aruco
import numpy as np
import os
import argparse
## TODO Make sure all imports are correct


## TODO Create the callback for the camera frame subscriber.
## TODO Call the write to flash function within this callback


def frame_callback(msg):
    if (rospy.Time.now() - msg.timestamp).to_sec() > 1/60:
        # rospy.loginfo(f"Skipping frame #{msg.seq}")
        return
    (frame, ids, pose) = detect_ArUco(msg.image)
    pub_frame.publish(frame)
    pub_sightings.publish(ids)
    pub_vector.publish(pose)


## Initialize the necessary nodes and the publishers.
rospy.init_node("cv_node")
pub_frame = rospy.Publisher("AnnotatedCameraFrame_topic", AnnotatedCameraFrame, queue_size=1)
## TODO Implement the publisher for the recent sightings.
pub_sightings = rospy.Publisher("RecentSightings_topic", RecentSighting, queue_size=1)
## TODO Implement the publisher for the pointing vector.
pub_vector = rospy.Publisher("PointingVector_topic", Header, queue_size=1)
## TODO Implement the subscriber for the camera frame.
sub_frame = rospy.Subscriber("RawCameraFrame_topic", Image, frame_callback)

## Initialize arUco marker detection
dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_6X6_50)
parameters =  cv2.aruco.DetectorParameters()
detector = cv2.aruco.ArucoDetector(dictionary, parameters)


## Begin the while loop to publish the AnnotatedCameraFrame message.
while not rospy.is_shutdown():
    rospy.spin()