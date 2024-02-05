"""
ARDVARC Flight Software

Author: Lyon Foster
email: lyon.foster@colorado.edu

Description:
    main state machine for all fsw

Notes:
    - Some things we NEVER want to do:
     + quit the program
     + do noting unintentionally
     + error out and quit the program
     + get stuck in a loop
"""

# local imports
from config.structures import STATES

# third part imports
import numpy as np
import rospy
from rosardvarc.msg import AnnotatedCameraFrame

# stl imports
import time

# TODO(LF): move whatever this ends up being to util
most_recent_message = None
def print_callback(message):
    print("bruh moment!")
    print(message)
    most_recent_message = message


"""Assignment of first state"""
main_state = STATES.FIND_1

"""Assignment of mission start time"""
mission_start_time = time.time()

rospy.init_node("fsm_node")
sub = rospy.Subscriber("AnnotatedCameraFrame_topic", AnnotatedCameraFrame, print_callback)

while(True):

    # TODO(LF): review state machine structure and design
    # ! Notional state machine example, not for flight use

    # Stuff that always happens
    pass

    if main_state == STATES.FIND_1:
        print("sleeping for 5 seconds")
        time.sleep(5)
        print("done sleeping, on to the next state")
        main_state = STATES.TRACK_1

    if main_state == STATES.TRACK_1:
        print("here's the most recent message: ")
        print(most_recent_message)

    # update rate of fsm
    time.sleep(1)
