import rospy
from ..config.topic_names import CAMERA_FRAMES
from sensor_msgs.msg import Image


rospy.init_node("camera")
pub = rospy.Publisher(CAMERA_FRAMES, Image, queue_size=1)
rate = rospy.Rate(60)
while not rospy.is_shutdown():
    now = rospy.Time.now()
    rospy.loginfo("Camera is publishing a frame")
    image_msg = Image(
        # TODO: Make this something reasonable
    )
    image_msg.header.stamp = now
    image_msg.data = bytearray(3*1000*1000)
    image_msg.height = 1000
    image_msg.width = 1000
    pub.publish(image_msg)
    rate.sleep()