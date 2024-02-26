""" 

ARDVARC Flight Software

Author: Tim Behrer
email: Timothy.Behrer@colorado.edu

Description: This is the CV node, it takes in the camera feed from ROS and annotates it for analysis, 
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
from .functional.process_frame import detect_ArUco_Direction_and_Pose, camera_frame_to_UAS_frame,my_estimatePoseSingleMarkers
import math
from ..config import constants
## Imports from the existing ArUco marker detection and annotation function
import sys
import cv2
from cv2 import aruco
import numpy as np
import os
import argparse
from cv_bridge import CvBridge
## TODO Make sure all imports are correct
## TODO Create the callback for the camera frame subscriber.
## TODO Call the write to flash function within this callback


## TODO (02/26 - TB) Finish the cv.py message publishing 

def frame_callback(msg: Image):
    now = rospy.Time.now()
    if (now - msg.header.stamp).to_sec() > 1/60:
        rospy.loginfo(f"CV skipped a frame ({now} vs {msg.header.stamp})")
        return
    frame = bridge.imgmsg_to_cv2(msg)

    detection_info = detect_ArUco_Direction_and_Pose(frame)
    if detection_info == None:
        rospy.loginfo("CV processed a frame but found nothing")
        return
    
    

    for id in detection_info.ids:
        if id in constants.ARUCO_ID2RGV_DICT.keys():
            detection_info.annotated_camera_frame
            annotated_frame = AnnotatedCameraFrame()
            annotated_frame.timestamp = rospy.Time.now
            annotated_frame.rgv_id = constants.ARUCO_ID2RGV_DICT[id]
            
            # TODO: Make this something reasonable based on detection_info.annotated_camera_frame (TB 2021-04-07: I think that this is right but im not sure that this is the correct way to do this)
            rospy.loginfo("CV published an annotated frame")
            pub_frame.publish(annotated_frame)

        sighting = RecentSighting(id)
        # TODO: Make this something reasonable based on id (TB 2021-04-07: I think that this is right but im not sure that this is the correct way to do this)
        rospy.loginfo("CV published an RGV sighting")
        pub_sightings.publish(sighting)

    for direction_vector in detection_info.direction_vectors:
        direction_vector_msg = UasToRgvDirectionVectorUasFrame(direction_vector)
        # TODO: Make this something reasonable based on direction_vector (TB 2024-02-26: I think that this is right but im not sure that this is the correct way to do this)
        rospy.loginfo("CV published a direction vector")
        pub_vector.publish(direction_vector_msg)


## Initialize the necessary nodes and the publishers.
rospy.init_node("cv_node")
bridge = CvBridge()
pub_frame = rospy.Publisher(ANNOTATED_CAMERA_FRAMES, AnnotatedCameraFrame, queue_size=1)
## TODO Implement the publisher for the recent sightings.
pub_sightings = rospy.Publisher(RECENT_RGV_SIGHTINGS, RecentSighting, queue_size=1)
## TODO Implement the publisher for the pointing vector.
pub_vector = rospy.Publisher(UAS_TO_RGV_DIRECTION_VECTORS, UasToRgvDirectionVectorUasFrame, queue_size=1)
## TODO Implement the subscriber for the camera frame.
sub_frame = rospy.Subscriber(CAMERA_FRAMES, Image, frame_callback)


## Spin until killed
rospy.spin()