import rospy
from std_msgs.msg import Header
import collections


uas_state_buffer = collections.deque([], 50)
direction_vector_buffer = collections.deque([], 50)


def setup():
    def uas_state_callback(msg):
        uas_state_buffer.appendleft(msg)


    def direction_vector_callback(msg):
        direction_vector_buffer.appendleft(msg)


    pub = rospy.Publisher("estimation/estimated_rgv_locations", Header, queue_size=1)
    uas_state_sub = rospy.Subscriber("pixhawk/uas_states", Header, uas_state_callback)
    direction_vector_sub = rospy.Subscriber("estimation/direction_vectors_uas", Header, direction_vector_callback)
    return pub