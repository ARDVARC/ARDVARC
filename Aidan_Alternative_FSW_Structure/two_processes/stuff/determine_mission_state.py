import rospy
from std_msgs.msg import Header


def setup():
    def callback(msg):
        rospy.loginfo("'Determine Mission State' is publishing a mission state")
        pub.publish(msg)


    pub = rospy.Publisher("main_state_machine/mission_states", Header, queue_size=1)
    sub = rospy.Subscriber("main_state_machine/state_machine_criteria", Header, callback)