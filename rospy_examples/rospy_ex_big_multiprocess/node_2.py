import rospy
import std_msgs.msg


def print_callback(message):
    print("Node 2 - Got a callback!:")
    print(message)
    pub.publish(
        std_msgs.msg.Int8(2)
    )


rospy.init_node("node_2")
pub = rospy.Publisher("topic_2", std_msgs.msg.Int8, queue_size=1)
sub = rospy.Subscriber("topic_1", std_msgs.msg.Int8, print_callback)
while not rospy.is_shutdown():
    rospy.spin()