import rospy
from std_msgs.msg import Header
from config.structures import MissionState


_current_state: MissionState = MissionState.FIND_RGV_1


def _state_machine_criteria_callback(msg):
    # Choose the next mission state based on _current_state and msg
    # new_state = math math math
    # _current_state = new_state
    
    # Publish new mission state
    _mission_state_pub.publish(msg)


_mission_state_pub = rospy.Publisher("main_state_machine/mission_states", Header, queue_size=1)
_state_machine_criteria_sub = rospy.Subscriber("main_state_machine/state_machine_criteria", Header, _state_machine_criteria_callback)