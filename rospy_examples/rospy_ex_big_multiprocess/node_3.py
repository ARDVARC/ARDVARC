import rospy
import std_msgs.msg


def print_callback(message):
    print("Node 3 - Got a callback!:")
    print(message)
    pub.publish(
        std_msgs.msg.Int8(3)
    )


rospy.init_node("node_3")
pub = rospy.Publisher("topic_3", std_msgs.msg.Int8, queue_size=1)
sub = rospy.Subscriber("topic_2", std_msgs.msg.Int8, print_callback)
while not rospy.is_shutdown():
    rospy.spin()