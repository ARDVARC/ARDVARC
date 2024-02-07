import rospy
import collections
from config.topic_names import STATE_MACHINE_CRITERIA, ESTIMATED_RGV_STATES, RECENT_RGV_SIGHTINGS, MISSION_STATES
from rosardvarc.msg import StateMachineCriteria, EstimatedRgvState, RecentSighting, MissionState


_sightings_buffer = collections.deque([], 50)
_mission_state_buffer = collections.deque([], 50)


def _estimated_rgv_state_callback(msg: EstimatedRgvState):
    # Determine state machine criteria
    # math math math
    
    # Publish state machine criteria
    _state_machine_criteria_pub.publish(
        StateMachineCriteria(
            # TODO: Make this something reasonable
        )
    )


def _sightings_callback(msg: RecentSighting):
    # Put in a cache
    _sightings_buffer.appendleft(msg)


def _mission_state_callback(msg: MissionState):
    # Put in a cache
    _mission_state_buffer.appendleft(msg)


_state_machine_criteria_pub = rospy.Publisher(STATE_MACHINE_CRITERIA, StateMachineCriteria, queue_size=1)
_estimated_rgv_state_sub = rospy.Subscriber(ESTIMATED_RGV_STATES, EstimatedRgvState, _estimated_rgv_state_callback)
_sightings_sub = rospy.Subscriber(RECENT_RGV_SIGHTINGS, RecentSighting, _sightings_callback)
_mission_state_sub = rospy.Subscriber(MISSION_STATES, MissionState, _mission_state_callback)