import rospy
import std_msgs.msg


def print_callback(message):
    print("Node 5 - Got a callback!:")
    print(message)
    pub.publish(
        std_msgs.msg.Int8(5)
    )


rospy.init_node("node_5")
pub = rospy.Publisher("topic_5", std_msgs.msg.Int8, queue_size=1)
sub = rospy.Subscriber("topic_4", std_msgs.msg.Int8, print_callback)
while not rospy.is_shutdown():
    rospy.spin()