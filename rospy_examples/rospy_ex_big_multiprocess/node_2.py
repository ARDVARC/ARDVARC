import rospy
import std_msgs.msg


# This callback will be executed whenever node_1 publishes a message
# to topic_1
def print_callback(message):
    print("Node 2 - Got a callback!:")
    print(message)
    pub.publish(
        std_msgs.msg.Int8(2)
    )


# Each process in this example has its own node - this one is node_2
rospy.init_node("node_2")
# This process/node will listen to the messages that node_1 writes to
# topic_1 and will write messages to topic_2.
pub = rospy.Publisher("topic_2", std_msgs.msg.Int8, queue_size=1)
sub = rospy.Subscriber("topic_1", std_msgs.msg.Int8, print_callback)

# Spin until killed
while not rospy.is_shutdown():
    rospy.spin()