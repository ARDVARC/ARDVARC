import rospy
from std_msgs.msg import Header
import collections


sightings_buffer = collections.deque([], 50)


def vel_callback(msg):
    rospy.loginfo("'Generate State Machine Criteria' is publishing state machine criteria")
    pub.publish(msg)


def pos_callback(msg):
    # I think this should really be in the same message as velocity because they
    # should be determined at the same time
    pass


def sightings_callback(msg):
    sightings_buffer.appendleft(msg)


def mission_state_callback(msg):
    # I also think this should be combined with 'Determine Misison State' to
    # avoid this mess
    pass


rospy.init_node("generate_state_machine_criteria")
pub = rospy.Publisher("main_state_machine/state_machine_criteria", Header, queue_size=1)
vel_sub = rospy.Subscriber("estimation/estimated_rgv_velocities", Header, vel_callback)
pos_sub = rospy.Subscriber("estimation/estimated_rgv_positions", Header, pos_callback)
sightings_sub = rospy.Subscriber("camera/rgv_sightings", Header, sightings_callback)
mission_state_sub = rospy.Subscriber("main_state_machine/mission_states", Header, mission_state_callback)

while not rospy.is_shutdown():
    rospy.spin()