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
"""




## Imports
import time
import rospy
import genpy
from rosardvarc.msg import AnnotatedCameraFrame
## TODO Import the necessary message types.
"""from rosardvarc.msg import RawCameraFrame""" ## This is the subscriber to the camera frame.
"""from rosardvarc.msg import RecentSighting""" ## This is the publisher for the recent sightings.
"""from rosardvarc.msg import Uas2RgvPointingVector""" ## This is the publisher for the pointing vector.
import math

## TODO Create the callback for the camera frame subscriber.
## TODO Call the write to flash fucntion within this callback
"""
def print_callback(message):
    print("bruh moment!")
    print(message)
"""


## Initialize the necessary nodes and the publishers.
rospy.init_node("cv_node")
pub_frame = rospy.Publisher("AnnotatedCameraFrame_topic", AnnotatedCameraFrame, queue_size=1)
## TODO Implement the publisher for the recent sightings.
"""pub_sightings = rospy.Publisher("RecentSightings_topic", RecentSighting, queue_size=1)"""
## TODO Implement the publisher for the pointing vector.
"""pub_vector = rospy.Publisher("PointingVector_topic", Uas2RgvPointingVector, queue_size=1)"""
## TODO Implement the subscriber for the camera frame.
"""sub_frame = rospy.Subscriber("RawCameraFrame_topic", RawCameraFrame, print_callback)"""

## Set the rate of the publisher.
rate = rospy.Rate(1)

## Begin the while loop to publish the AnnotatedCameraFrame message.
while not rospy.is_shutdown():
    ## Get the time
    now = time.time()
    now_sec = int(math.floor(now))
    now_nano = int((now-math.floor(now))*1e9)
    ## Publish the AnnotatedCameraFrame message.
    pub_frame.publish(
        AnnotatedCameraFrame(
            timestamp=genpy.Time(now_sec, now_nano),
            rgv_id=AnnotatedCameraFrame.RGV_1,
            annotated_image=69
        )
    )
    ## Sleep for the rate.
    rate.sleep()
