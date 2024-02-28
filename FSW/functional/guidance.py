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

# Clients for arming?
rospy.wait_for_service(CLIENT_ARMING)
arming_client = rospy.ServiceProxy(CLIENT_ARMING, CommandBool)

rospy.wait_for_service(CLIENT_SET_MODE)
set_mode_client = rospy.ServiceProxy(CLIENT_SET_MODE, SetMode)

# must be greater than 2 Hz however it ends up getting implemented
guidance_update_rate = rospy.Rate(20)


offb_set_mode = SetModeRequest()
offb_set_mode.custom_mode = 'OFFBOARD'

arm_cmd = CommandBoolRequest()
arm_cmd.value = True


current_UAS_arming_state = State()

current_UAS_pose = PoseStamped()

current_RGV_state_lla = NavSatFix()

last_req = rospy.Time.now()

# this is the setpoint that will be written
toBeWritten_setpoint = PoseStamped()

def _UAS_arming_state_callback(msg: State):
    global current_UAS_arming_state
    current_UAS_arming_state = msg

def _estimated_rgv_state_callback(msg: NavSatFix):
    rospy.loginfo("Guidance saved an estimated RGV state")
    rospy.logdebug(msg)
    
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
    x_set, y_set, z_set = _calc_orbit_setpoint(rgv, uas, t)
    

    # overwrite the global version
    global toBeWritten_setpoint
    toBeWritten_setpoint.pose.position.x = x_set
    toBeWritten_setpoint.pose.position.y = y_set
    toBeWritten_setpoint.pose.position.z = z_set

    # moved to the timer callback
    #_setpoint_pub.publish(toBeWritten_setpoint)

def _uas_pose_callback(msg: PoseStamped):
    rospy.loginfo("Guidance saved a UAS pose")

    global current_UAS_pose
    current_UAS_pose = msg

def _timer_callback(event=None):
    global last_req
    if(rospy.is_shutdown()): # then leave bro
        return
    else: # publish the setpoint after doing some checking
        if(current_UAS_arming_state.mode != "OFFBOARD" and (rospy.Time.now() - last_req) > rospy.Duration(5.0)):
            if(set_mode_client.call(offb_set_mode).mode_sent == True):
                rospy.loginfo("OFFBOARD enabled")

            last_req = rospy.Time.now()
        else:
            if(not current_UAS_arming_state.armed and (rospy.Time.now() - last_req) > rospy.Duration(5.0)):
                if(arming_client.call(arm_cmd).success == True):
                    rospy.loginfo("Vehicle armed")

                last_req = rospy.Time.now()

        rospy.loginfo("Guidance published an orbit setpoint")
        _setpoint_pub.publish(toBeWritten_setpoint)

# _setpoint_pub = rospy.Publisher(SETPOINTS, Setpoint, queue_size=1)
#_roi_pub = rospy.Publisher(REGIONS_OF_INTEREST, RegionOfInterest, queue_size=1)
#_estimated_rgv_state_sub = rospy.Subscriber(ESTIMATED_RGV_STATES, EstimatedRgvState, _estimated_rgv_state_callback)


# make the timer?? real time?? Ahh manmade multithreading horrors beyond my comprehension
rospy.Timer(rospy.Duration(0.05), _timer_callback)

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
    

    new_y = 5 

    return (2,new_y, 2)
