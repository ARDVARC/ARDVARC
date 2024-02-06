import rospy
import std_msgs.msg


# This callback will be executed whenever node_3 publishes a message
# to topic_3
def print_callback(message):
    print("Node 4 - Got a callback!:")
    print(message)
    pub.publish(
        std_msgs.msg.Int8(4)
    )


# Each process in this example has its own node - this one is node_4
rospy.init_node("node_4")
# This process/node will listen to the messages that node_3 writes to
# topic_3 and will write messages to topic_4.
pub = rospy.Publisher("topic_4", std_msgs.msg.Int8, queue_size=1)
sub = rospy.Subscriber("topic_3", std_msgs.msg.Int8, print_callback)

# Spin until killed
while not rospy.is_shutdown():
    rospy.spin()