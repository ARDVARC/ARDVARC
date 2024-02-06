import rospy
import std_msgs.msg


# This callback will be executed whenever node_5 publishes a message
# to topic_5
def print_callback(message):
    print("Node 1 - Got a callback!:")
    print(message)
    pub.publish(
        std_msgs.msg.Int8(1)
    )


# Each process in this example has its own node - this one is node_1
rospy.init_node("node_1")
# This process/node will listen to the messages that node_5 writes to
# topic_5 and will write messages to topic_1.
pub = rospy.Publisher("topic_1", std_msgs.msg.Int8, queue_size=1)
sub = rospy.Subscriber("topic_5", std_msgs.msg.Int8, print_callback)

# To kickstart the loop of messages, this process will publish one
# message to topic_1
rospy.sleep(3)
pub.publish(
    std_msgs.msg.Int8(1)
)

# Spin until killed
while not rospy.is_shutdown():
    rospy.spin()