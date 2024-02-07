import rospy
from std_msgs.msg import Header
import collections


rgv_positions_buffer = collections.deque([], 50)


def pos_callback(msg):
    rgv_positions_buffer.appendleft(msg)
    pass

def mission_state_callback(msg):
    rospy.loginfo("'determine_setpoint_and_roi' is publishing setpoint and roi")
    setpoint_pub.publish(msg)
    roi_pub.publish(msg)
    pass


rospy.init_node("determine_setpoint_and_roi")
setpoint_pub = rospy.Publisher("pixhawk/setpoints", Header, queue_size=1)
roi_pub = rospy.Publisher("pixhawk/rois", Header, queue_size=1)
pos_sub = rospy.Subscriber("estimation/estimated_rgv_positions", Header, pos_callback)
mission_state_sub = rospy.Subscriber("main_state_machine/mission_states", Header, mission_state_callback)

while not rospy.is_shutdown():
    rospy.spin()