import time
import rospy
import genpy
from rosardvarc.msg import AnnotatedCameraFrame
import math

def print_callback(message):
    print("bruh moment!")
    print(message)


rospy.init_node("cv_node")
pub = rospy.Publisher("AnnotatedCameraFrame_topic", AnnotatedCameraFrame, queue_size=1)
sub = rospy.Subscriber("AnnotatedCameraFrame_topic", AnnotatedCameraFrame, print_callback)
rate = rospy.Rate(1)

while not rospy.is_shutdown():
    now = time.time()
    now_sec = int(math.floor(now))
    now_nano = int((now-math.floor(now))*1e9)
    pub.publish(
        AnnotatedCameraFrame(
            timestamp=genpy.Time(now_sec, now_nano),
            rgv_id=AnnotatedCameraFrame.RGV_1,
            annotated_image=69
        )
    )
    rate.sleep()
