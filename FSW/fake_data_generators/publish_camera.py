import rospy
from ..config.topic_names import CAMERA_FRAMES
from sensor_msgs.msg import Image


rospy.init_node("camera")
pub = rospy.Publisher(CAMERA_FRAMES, Image, queue_size=1)
rate = rospy.Rate(60)
while not rospy.is_shutdown():
    now = rospy.Time.now()
    pub.publish(
        Image(
            # TODO: Make this something reasonable
        )
    )
    # rospy.loginfo("Camera is publishing frame")
    rate.sleep()