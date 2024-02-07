import rospy
from std_msgs.msg import Header
import collections


_sightings_buffer = collections.deque([], 50)


def _estimated_rgv_state_callback(msg):
    # Determine state machine criteria
    # math math math
    
    # Publish state machine criteria
    _state_machine_criteria_pub.publish(msg)


def _sightings_callback(msg):
    # Put in a cache
    _sightings_buffer.appendleft(msg)


def _mission_state_callback(msg):
    # Put in a cache
    pass


_state_machine_criteria_pub = rospy.Publisher("main_state_machine/state_machine_criteria", Header, queue_size=1)
_estimated_rgv_state_sub = rospy.Subscriber("estimation/estimated_rgv_states", Header, _estimated_rgv_state_callback)
_sightings_sub = rospy.Subscriber("camera/rgv_sightings", Header, _sightings_callback)
_mission_state_sub = rospy.Subscriber("main_state_machine/mission_states", Header, _mission_state_callback)