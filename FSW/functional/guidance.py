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
from std_msgs.msg import Header


def _pos_callback(msg):
    # Probably save to a buffer?
    pass

def _mission_state_callback(msg):
    # Do some stuff to prepare calc_orbit_setpoint
    rgv = None
    uas = None
    t = None
    
    # Call calc_orbit_setpoint
    orbit_setpoint = _calc_orbit_setpoint(rgv, uas, t)
    # roi = calc_roi(???)
    
    # Publish the setpoint and ROI
    _setpoint_pub.publish(orbit_setpoint)
    # roi_pub.publish(roi)


_setpoint_pub = rospy.Publisher("pixhawk/setpoints", Header, queue_size=1)
_roi_pub = rospy.Publisher("pixhawk/rois", Header, queue_size=1)
_pos_sub = rospy.Subscriber("estimation/estimated_rgv_positions", Header, _pos_callback)
_mission_state_sub = rospy.Subscriber("main_state_machine/mission_states", Header, _mission_state_callback)


def _calc_orbit_setpoint(RGV, UAS, t):
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


    pass
