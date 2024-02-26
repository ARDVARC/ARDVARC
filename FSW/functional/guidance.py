"""
ARDVARC Flight Software

Author: Lyon Foster
email: lyon.foster@colorado.edu

Description:
    File for guidance functions

Notes:
    - 
"""

import rospy
from rosardvarc.msg import Setpoint, RegionOfInterest, EstimatedRgvState, MissionState
from geometry_msgs.msg import PoseStamped
from genpy import Time
#from ..config.topic_names import SETPOINTS, REGIONS_OF_INTEREST, ESTIMATED_RGV_STATES, MISSION_STATES, UAS_POSES, MAVROS_GPS_POS_FORTESTING 
from ..config.topic_names import *
import collections
from typing import Deque
from ..config.constants import *
# import ..config.constants as const

# LYON used this message for testing
from sensor_msgs.msg import NavSatFix

# Needed for io with mavros
from mavros_msgs.msg import State
from mavros_msgs.srv import CommandBool, CommandBoolRequest, SetMode, SetModeRequest



## TODO- Probably no need to keep more than one?
#_estimated_rgv_state_buffer: Deque[EstimatedRgvState] = collections.deque([], 50)
## TODO- Probably no need to keep more than one?
#_uas_pose_buffer: Deque[PoseStamped] = collections.deque([], 50)
#

#def _estimated_rgv_state_callback(msg: EstimatedRgvState):
#    rospy.loginfo("Guidance saved an estimated RGV state")
#    _estimated_rgv_state_buffer.appendleft(msg)
#
#    pass
#


current_UAS_arming_state = State()

current_UAS_pose = PoseStamped()

current_RGV_state_lla = NavSatFix()

def _UAS_arming_state_callback(msg: State):
    global current_UAS_arming_state
    current_UAS_arming_state = msg

def _estimated_rgv_state_callback(msg: NavSatFix):
    rospy.loginfo("Guidance saved an estimated RGV state")
    rospy.loginfo(msg)
    
    global current_RGV_state_lla
    current_RGV_state_lla = msg

def _mission_state_callback(msg: MissionState):
    # Do some stuff to prepare calc_orbit_setpoint
    # TODO - This is just an example:
#    if len(_estimated_rgv_state_buffer) == 0 or len(_uas_pose_buffer) == 0:
#        rospy.loginfo("Guidance ignored a mission state update")
#        return

    rgv = current_RGV_state_lla
    uas = current_UAS_pose
    t = msg.timestamp
    # Probably also want to care about the new mission state in msg
    
    # Call calc_orbit_setpoint
    orbit_setpoint = _calc_orbit_setpoint(rgv, uas, t)
    # roi = calc_roi(???)
    
    # Publish the setpoint and ROI
    rospy.loginfo("Guidance published an orbit setpoint")

    #_setpoint_pub.publish(orbit_setpoint)
    # _roi_pub.publish(roi)

def _uas_pose_callback(msg: PoseStamped):
    rospy.loginfo("Guidance saved a UAS pose")

    global current_UAS_pose
    current_UAS_pose = msg


# _setpoint_pub = rospy.Publisher(SETPOINTS, Setpoint, queue_size=1)
#_roi_pub = rospy.Publisher(REGIONS_OF_INTEREST, RegionOfInterest, queue_size=1)
#_estimated_rgv_state_sub = rospy.Subscriber(ESTIMATED_RGV_STATES, EstimatedRgvState, _estimated_rgv_state_callback)

# make all subs and doms (jk lol pubs)
_setpoint_pub = rospy.Publisher(UAS_SETPOINT_LOCAL, PoseStamped, queue_size=10)
_estimated_rgv_state_sub = rospy.Subscriber(MAVROS_GPS_POS_FORTESTING, NavSatFix, _estimated_rgv_state_callback)
_mission_state_sub = rospy.Subscriber(MISSION_STATES, MissionState, _mission_state_callback)
_uas_pose_sub = rospy.Subscriber(UAS_POSES, PoseStamped, _uas_pose_callback)
_uas_arming_state_sub = rospy.Subscriber(UAS_ARMING_STATE, State,_UAS_arming_state_callback)

# Clients for arming?
rospy.wait_for_service(CLIENT_ARMING)
arming_client = rospy.ServiceProxy(CLIENT_ARMING, CommandBool)

rospy.wait_for_service(CLIENT_SET_MODE)
set_mode_client = rospy.ServiceProxy(CLIENT_SET_MODE, SetMode)




def _calc_orbit_setpoint(RGV: EstimatedRgvState, UAS: PoseStamped, t: Time) -> Setpoint:
    """ Calculates the orbit set point

    This function takes the RGV & UAS states, as well as time to calculate a 
    setpoint that can be given to the Pixhawk.

    Args:
        RGV: ROS message type containing RGV state information
        UAS: ROS message type containing UAS state information
        t: time elapsed since start of mission time

    Returns: 
        orbit_setpoint: Struct (TBR) containing orbit setpoint data

    Raises:
        None: Raises None at the moment (TBR)
    """

    # math math math
    
    return Setpoint(
        # TODO: Make this something reasonable
    )
