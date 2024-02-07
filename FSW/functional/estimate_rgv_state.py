import rospy
from std_msgs.msg import Header
import collections


_uas_state_buffer = collections.deque([], 50)
_direction_vector_buffer = collections.deque([], 50)


def _uas_state_callback(msg):
    _uas_state_buffer.appendleft(msg)


def _direction_vector_callback(msg):
    _direction_vector_buffer.appendleft(msg)


_estimated_rgv_state_pub = rospy.Publisher("estimation/estimated_rgv_states", Header, queue_size=1)
_uas_state_sub = rospy.Subscriber("pixhawk/uas_states", Header, _uas_state_callback)
_direction_vector_sub = rospy.Subscriber("estimation/direction_vectors_uas", Header, _direction_vector_callback)


def estimate_rgv_state():
    """
    Called by main.py in the main loop.
    Estimates the RGV state.
    """
    # Estimate the RGV state
    # math math math
    
    # Publish estimate
    _estimated_rgv_state_pub.publish(None)