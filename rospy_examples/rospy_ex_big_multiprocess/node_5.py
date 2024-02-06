import rospy
import std_msgs.msg


# This callback will be executed whenever node_4 publishes a message
# to topic_4
def print_callback(message):
    print("Node 5 - Got a callback!:")
    print(message)
    pub.publish(
        std_msgs.msg.Int8(5)
    )


# Each process in this example has its own node - this one is node_5
rospy.init_node("node_5")
# This process/node will listen to the messages that node_4 writes to
# topic_4 and will write messages to topic_5.
pub = rospy.Publisher("topic_5", std_msgs.msg.Int8, queue_size=1)
sub = rospy.Subscriber("topic_4", std_msgs.msg.Int8, print_callback)

# Spin until killed
while not rospy.is_shutdown():
    rospy.spin()