import rospy
import collections
import math
import numpy as np
from ..config.topic_names import ESTIMATED_RGV_STATES, UAS_POSES, UAS_TO_RGV_DIRECTION_VECTORS
from rosardvarc.msg import EstimatedRgvState, UasToRgvDirectionVectorUasFrame
from geometry_msgs.msg import PoseStamped


_uas_state_buffer = collections.deque([], 50)
_direction_vector_buffer = collections.deque([], 50)


def _uas_state_callback(msg: PoseStamped):
    _uas_state_buffer.appendleft(msg)
    rospy.loginfo("RGV state estimator saved UAS state")


def _direction_vector_callback(msg: UasToRgvDirectionVectorUasFrame):
    _direction_vector_buffer.appendleft(msg)
    rospy.loginfo("RGV state estimator saved direction vector")


_estimated_rgv_state_pub = rospy.Publisher(ESTIMATED_RGV_STATES, EstimatedRgvState, queue_size=1)
_uas_state_sub = rospy.Subscriber(UAS_POSES, PoseStamped, _uas_state_callback)
_direction_vector_sub = rospy.Subscriber(UAS_TO_RGV_DIRECTION_VECTORS, UasToRgvDirectionVectorUasFrame, _direction_vector_callback)


def estimate_rgv_state():
    """
    Called by main.py in the main loop.
    Estimates and publishes the RGV state.
    """
    # lat lon to ecef (treat ecef as inertial frame)
    def lla_to_ecef(lat, lon, alt):
        # Convert to radians
        rad_lat = lat * (math.pi / 180.0)
        rad_lon = lon * (math.pi / 180.0)

        # Account for shape of earth
        a = 6378137 # m
        finv = 298.257223563
        f = 1 / finv
        e2 = 1 - (1-f) * (1-f)
        v = a / math.sqrt(1-e2 * math.sin(rad_lat) * math.sin(rad_lat))
        # Angle math
        x = (v + alt) * math.cos(rad_lat) * math.cos(rad_lon)
        y = (v + alt) * math.cos(rad_lat) * math.sin(rad_lon)
        z = (v * (1-e2) + alt) * math.sin(rad_lat)

        return x, y, z

    # Estimate the RGV state

    # Bring in vars from messages
    ### TODO - MESSAGES CHANGING
    # Convert to ecef for ease of computations
    # [uas_state_x, uas_state_y, uas_alt]lla_to_ecef(lat, lon, alt)
    # as_to_rgv_az =   # [0, 2pi] check with rob
    # as_to_rgv_el  l =   # [-pi/2, pi/2]
    # # UAS yaw angle
    # uas_yaw  = 
    # # UAS height above ground 
    # uas_height = uas_alt - uas_starting_alt
    # # Assume azimuth is always in line with UAS
    # # Elevation angle bluetooth sensor is at (straight down = 0) (rad)
    # # Hardcode for now 
    bluetooth_angle = 0 
    
    # Find rectangular coords
    # Vector length from uas to rgv
    rho = uas_height/math.cos(uas_to_rgv_el)
    # Angle math for relative locations in inertial coords
    uas_to_rgv_x = rho*math.sin(uas_to_rgv_az - uas_yaw)*math.cos(as_to_rgv_el - bluetooth_angle)
    uas_to_rgv_y = -rho*math.sin(uas_to_rgv_az - uas_yaw)*math.sin(as_to_rgv_el - bluetooth_angle)
    # Convert relative to absolute
    rgv_state_x = uas_state_x + uas_to_rgv_x
    rgv_state_y = uas_state_y + uas_to_rgv_y
    
    # Publish estimate
    rospy.loginfo("RGV state estimator published an RGV state estimate")
    _estimated_rgv_state_pub.publish(
        EstimatedRgvState(
            # TODO: Make this something reasonable
        )
    )