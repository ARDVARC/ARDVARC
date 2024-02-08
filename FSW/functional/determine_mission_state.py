import rospy
from ..config.structures import MissionStates
from ..config.topic_names import MISSION_STATES, STATE_MACHINE_CRITERIA
from rosardvarc.msg import MissionState, StateMachineCriteria


_current_state: MissionStates = MissionStates.FIND_RGV_1


def _state_machine_criteria_callback(msg: StateMachineCriteria):
    # Choose the next mission state based on _current_state and msg
    # new_state = math math math
    # _current_state = new_state
    
    # Publish new mission state
    _mission_state_pub.publish(
        MissionState(
            # TODO: Make this something reasonable
        )
    )


_mission_state_pub = rospy.Publisher(MISSION_STATES, MissionState, queue_size=1)
_state_machine_criteria_sub = rospy.Subscriber(STATE_MACHINE_CRITERIA, StateMachineCriteria, _state_machine_criteria_callback)