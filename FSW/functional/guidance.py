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

# Clients for arming?
rospy.wait_for_service(CLIENT_ARMING)
arming_client = rospy.ServiceProxy(CLIENT_ARMING, CommandBool)

rospy.wait_for_service(CLIENT_SET_MODE)
set_mode_client = rospy.ServiceProxy(CLIENT_SET_MODE, SetMode)

# must be greater than 2 Hz however it ends up getting implemented
guidance_update_rate = rospy.Rate(20)

current_UAS_arming_state = State()

current_UAS_pose = PoseStamped()

current_RGV_state_lla = NavSatFix()

last_req = rospy.Time.now()

# this is the setpoint that will be written
toBeWritten_setpoint = PoseStamped()

_setpoint_pub: rospy.Publisher
_estimated_rgv_state_sub: rospy.Subscriber
_mission_state_sub: rospy.Subscriber
_uas_pose_sub: rospy.Subscriber
_uas_arming_state_sub: rospy.Subscriber


offboard_start_time = None


def _UAS_arming_state_callback(msg: State):
    global current_UAS_arming_state
    current_UAS_arming_state = msg

def _estimated_rgv_state_callback(msg: NavSatFix):
    rospy.logdebug("Guidance saved an estimated RGV state")
    rospy.logdebug(msg)
    
    global current_RGV_state_lla
    current_RGV_state_lla = msg

def _mission_state_callback(msg: MissionState):
    # Do some stuff to prepare calc_orbit_setpoint
    rgv = current_RGV_state_lla
    uas = current_UAS_pose
    t = msg.timestamp
    # Probably also want to care about the new mission state in msg
    
def _uas_pose_callback(msg: PoseStamped):
    rospy.logdebug("Guidance saved a UAS pose")

    global current_UAS_pose
    current_UAS_pose = msg

def _timer_callback(event=None):
    global offboard_start_time
    if(rospy.is_shutdown()): # then leave bro
        return
    else:
        # bool for if we're offboard or not
        offboard_status = (current_UAS_arming_state.mode == "OFFBOARD")
        if offboard_status and offboard_start_time is None:
            offboard_start_time = rospy.Time.now()
        
        # Call calc_orbit_setpoint
        x_set, y_set, z_set = _calc_orbit_setpoint(0,0, offboard_start_time, offboard_status)
         
        current_setpoint = PoseStamped()
        # set the fields of 
        current_setpoint.pose.position.x = x_set
        current_setpoint.pose.position.y = y_set
        current_setpoint.pose.position.z = z_set

   
        _setpoint_pub.publish(current_setpoint)
        rospy.logdebug(f"Guidance published an orbit setpoint: {current_setpoint}")


# make the timer?? real time?? Ahh manmade multithreading horrors beyond my comprehension
rospy.Timer(rospy.Duration(0.05), _timer_callback)

def setup():
    """
    Setup publishers and subscribers for guidance.py
    """
    
    global _setpoint_pub, _estimated_rgv_state_sub, _mission_state_sub, _uas_pose_sub, _uas_arming_state_sub
 
    # make all subs and pubs
    _setpoint_pub = rospy.Publisher(UAS_SETPOINT_LOCAL, PoseStamped, queue_size=10)
    _estimated_rgv_state_sub = rospy.Subscriber(MAVROS_GPS_POS_FORTESTING, NavSatFix, _estimated_rgv_state_callback)
    _mission_state_sub = rospy.Subscriber(MISSION_STATES, MissionState, _mission_state_callback)
    _uas_pose_sub = rospy.Subscriber(UAS_POSES, PoseStamped, _uas_pose_callback)
    _uas_arming_state_sub = rospy.Subscriber(UAS_ARMING_STATE, State,_UAS_arming_state_callback)

    # Send a few setpoints before starting
    # because can't switch to offboard until after some setpoints have been given
    for i in range(100):
        if(rospy.is_shutdown()):
            break
        #TODO(LF) make this publish the current UAS state for the first however many commands
        dummy_set_point = PoseStamped()
        dummy_set_point.pose.position.x = 0
        dummy_set_point.pose.position.y = 0
        dummy_set_point.pose.position.z = 0

        rate = rospy.Rate(20)
        _setpoint_pub.publish(dummy_set_point)
        rate.sleep()


def _calc_orbit_setpoint(RGV: EstimatedRgvState, UAS: PoseStamped, start_time: rospy.Time, offboard_status: bool) -> list:
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
    
    now = rospy.Time.now()

    if offboard_status:
        if now > start_time + rospy.Duration(5) and now < start_time + rospy.Duration(10):
            setpoint = [2,0,2]
        elif now > start_time + rospy.Duration(10):
            setpoint = [-2,0,2]
        else:
            setpoint = DEFAULT_SETPOINT
            rospy.logdebug("null setpoint returned")
    else:
        setpoint = DEFAULT_SETPOINT

    return setpoint
