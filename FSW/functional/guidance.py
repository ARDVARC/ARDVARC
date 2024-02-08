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
from ..config.topic_names import SETPOINTS, REGIONS_OF_INTEREST, ESTIMATED_RGV_STATES, MISSION_STATES, UAS_POSES
import collections
from typing import Deque


# TODO- Probably no need to keep more than one?
_estimated_rgv_state_buffer: Deque[EstimatedRgvState] = collections.deque([], 50)
# TODO- Probably no need to keep more than one?
_uas_pose_buffer: Deque[PoseStamped] = collections.deque([], 50)


def _estimated_rgv_state_callback(msg: EstimatedRgvState):
    _estimated_rgv_state_buffer.appendleft(msg)
    pass

def _mission_state_callback(msg: MissionState):
    # Do some stuff to prepare calc_orbit_setpoint
    # TODO - This is just an example:
    if len(_estimated_rgv_state_buffer) == 0 or len(_uas_pose_buffer) == 0:
        return
    rgv = _estimated_rgv_state_buffer[0]
    uas = _uas_pose_buffer[0]
    t = msg.timestamp
    # Probably also want to care about the new mission state in msg
    
    # Call calc_orbit_setpoint
    orbit_setpoint = _calc_orbit_setpoint(rgv, uas, t)
    # roi = calc_roi(???)
    
    # Publish the setpoint and ROI
    _setpoint_pub.publish(orbit_setpoint)
    # _roi_pub.publish(roi)

def _uas_pose_callback(msg: PoseStamped):
    _uas_pose_buffer.appendleft(msg)
    pass

_setpoint_pub = rospy.Publisher(SETPOINTS, Setpoint, queue_size=1)
_roi_pub = rospy.Publisher(REGIONS_OF_INTEREST, RegionOfInterest, queue_size=1)
_estimated_rgv_state_sub = rospy.Subscriber(ESTIMATED_RGV_STATES, EstimatedRgvState, _estimated_rgv_state_callback)
_mission_state_sub = rospy.Subscriber(MISSION_STATES, MissionState, _mission_state_callback)
_uas_pose_sub = rospy.Subscriber(UAS_POSES, PoseStamped, _uas_pose_callback)


def _calc_orbit_setpoint(RGV: EstimatedRgvState, UAS: PoseStamped, t: Time) -> Setpoint:
    """ Calculates the orbit set point

    This function takes the RGV & UAS states, as well as time to calculate a 
    setpoint that can be given to the Pixhawk.

    Args:
        RGV: struct containing RGV state information
        UAS: struct containing UAS state information
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