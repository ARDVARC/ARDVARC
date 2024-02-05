import rospy
import std_msgs.msg


def print_callback(message):
    print("Node 1 - Got a callback!:")
    print(message)
    pub.publish(
        std_msgs.msg.Int8(1)
    )


rospy.init_node("node_1")
pub = rospy.Publisher("topic_1", std_msgs.msg.Int8, queue_size=1)
sub = rospy.Subscriber("topic_5", std_msgs.msg.Int8, print_callback)
rospy.sleep(3)
pub.publish(
    std_msgs.msg.Int8(1)
)
while not rospy.is_shutdown():
    rospy.spin()