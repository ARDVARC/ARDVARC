import rospy
import std_msgs.msg


# This callback will be executed whenever node_2 publishes a message
# to topic_2
def print_callback(message):
    print("Node 3 - Got a callback!:")
    print(message)
    pub.publish(
        std_msgs.msg.Int8(3)
    )


# Each process in this example has its own node - this one is node_3
rospy.init_node("node_3")
# This process/node will listen to the messages that node_2 writes to
# topic_2 and will write messages to topic_3.
pub = rospy.Publisher("topic_3", std_msgs.msg.Int8, queue_size=1)
sub = rospy.Subscriber("topic_2", std_msgs.msg.Int8, print_callback)

# Spin until killed
while not rospy.is_shutdown():
    rospy.spin()