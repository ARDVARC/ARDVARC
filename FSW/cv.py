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
from .config.topic_names import ANNOTATED_CAMERA_FRAMES, RECENT_RGV_SIGHTINGS, UAS_TO_RGV_DIRECTION_VECTORS, CAMERA_FRAMES
from rosardvarc.msg import AnnotatedCameraFrame, RecentSighting, UasToRgvDirectionVectorUasFrame
from sensor_msgs.msg import Image
from .functional.process_frame import detect_ArUco
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


def frame_callback(msg: Image):
    now = rospy.Time.now()
    if (now - msg.header.stamp).to_sec() > 1/60:
        rospy.loginfo(f"CV skipped a frame ({now} vs {msg.header.stamp})")
        return
    data = np.frombuffer(msg.data, dtype=np.uint8)
    frame = np.reshape(data, (msg.width, msg.height, 3))
    detection_info = detect_ArUco(frame)
    if detection_info == None:
        rospy.loginfo("CV processed a frame but found nothing")
        return
    annotated_frame = AnnotatedCameraFrame(
        # TODO: Make this something reasonable based on detection_info.annotated_camera_frame
    )
    rospy.loginfo("CV published an annotated frame")
    pub_frame.publish(annotated_frame)
    for id in detection_info.ids:
        sighting = RecentSighting(
            # TODO: Make this something reasonable based on id
        )
        rospy.loginfo("CV published an RGV sighting")
        pub_sightings.publish(sighting)
    for direction_vector in detection_info.direction_vectors:
        direction_vector_msg = UasToRgvDirectionVectorUasFrame(
            # TODO: Make this something reasonable based on direction_vector
        )
        rospy.loginfo("CV published a direction vector")
        pub_vector.publish(direction_vector_msg)


## Initialize the necessary nodes and the publishers.
rospy.init_node("cv_node")
pub_frame = rospy.Publisher(ANNOTATED_CAMERA_FRAMES, AnnotatedCameraFrame, queue_size=1)
## TODO Implement the publisher for the recent sightings.
pub_sightings = rospy.Publisher(RECENT_RGV_SIGHTINGS, RecentSighting, queue_size=1)
## TODO Implement the publisher for the pointing vector.
pub_vector = rospy.Publisher(UAS_TO_RGV_DIRECTION_VECTORS, UasToRgvDirectionVectorUasFrame, queue_size=1)
## TODO Implement the subscriber for the camera frame.
sub_frame = rospy.Subscriber(CAMERA_FRAMES, Image, frame_callback)


## Spin until killed
rospy.spin()