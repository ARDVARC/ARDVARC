import rospy
import std_msgs.msg


# These are a bunch of different callbacks that each subscriber uses.
# This is probably not a very good way to do this but it works for
# the example. 
def print_callback_1(message):
    print("Node 1 - Got a callback!:")
    print(message)
    pub1.publish(
        std_msgs.msg.Int8(1)
    )


def print_callback_2(message):
    print("Node 2 - Got a callback!:")
    print(message)
    pub2.publish(
        std_msgs.msg.Int8(2)
    )


def print_callback_3(message):
    print("Node 3 - Got a callback!:")
    print(message)
    pub3.publish(
        std_msgs.msg.Int8(3)
    )


def print_callback_4(message):
    print("Node 4 - Got a callback!:")
    print(message)
    pub4.publish(
        std_msgs.msg.Int8(4)
    )


def print_callback_5(message):
    print("Node 5 - Got a callback!:")
    print(message)
    pub5.publish(
        std_msgs.msg.Int8(5)
    )


# This process will just have one node but it will have a bunch of different
# publishers and subscribers talking to each other over different topics
rospy.init_node("ex_node")
pub1 = rospy.Publisher("topic_1", std_msgs.msg.Int8, queue_size=1)
sub1 = rospy.Subscriber("topic_5", std_msgs.msg.Int8, print_callback_1)
pub2 = rospy.Publisher("topic_2", std_msgs.msg.Int8, queue_size=1)
sub2 = rospy.Subscriber("topic_1", std_msgs.msg.Int8, print_callback_2)
pub3 = rospy.Publisher("topic_3", std_msgs.msg.Int8, queue_size=1)
sub3 = rospy.Subscriber("topic_2", std_msgs.msg.Int8, print_callback_3)
pub4 = rospy.Publisher("topic_4", std_msgs.msg.Int8, queue_size=1)
sub4 = rospy.Subscriber("topic_3", std_msgs.msg.Int8, print_callback_4)
pub5 = rospy.Publisher("topic_5", std_msgs.msg.Int8, queue_size=1)
sub5 = rospy.Subscriber("topic_4", std_msgs.msg.Int8, print_callback_5)
rate = rospy.Rate(1)

# To kickstart the loop, one message is sent to topic_1
rospy.sleep(3)
pub1.publish(
    std_msgs.msg.Int8(1)
)

# Spin until killed
while not rospy.is_shutdown():
    rospy.spin()