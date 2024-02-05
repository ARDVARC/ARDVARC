import rospy
import std_msgs.msg


def print_callback(message):
    print("Node 4 - Got a callback!:")
    print(message)
    pub.publish(
        std_msgs.msg.Int8(4)
    )


rospy.init_node("node_4")
pub = rospy.Publisher("topic_4", std_msgs.msg.Int8, queue_size=1)
sub = rospy.Subscriber("topic_3", std_msgs.msg.Int8, print_callback)
while not rospy.is_shutdown():
    rospy.spin()