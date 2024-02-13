import rospy
import collections
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
    # Estimate the RGV state
    # math math math
    
    # Publish estimate
    rospy.loginfo("RGV state estimator published an RGV state estimate")
    _estimated_rgv_state_pub.publish(
        EstimatedRgvState(
            # TODO: Make this something reasonable
        )
    )