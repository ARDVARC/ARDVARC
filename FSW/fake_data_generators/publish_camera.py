import rospy
from ..config.topic_names import CAMERA_FRAMES
from sensor_msgs.msg import Image
import cv2
from cv_bridge import CvBridge


def restart_video():
    cap = cv2.VideoCapture("FSW/fake_data_generators/DJI_0011_AR_2_30_S_-_Trim.mp4")
    if (cap.isOpened()== False): 
        print("Error opening video file") 
    return cap


rospy.init_node("camera")
pub = rospy.Publisher(CAMERA_FRAMES, Image, queue_size=1)
rate = rospy.Rate(60)
bridge = CvBridge()

cap = restart_video()

while not rospy.is_shutdown():
    ret, frame = cap.read()
    
    if not ret:
        cap = restart_video()
        continue
    
    frame_as_bytes = cv2.imencode('.jpg', frame)[1].tobytes()
    now = rospy.Time.now()
    
    image_msg = bridge.cv2_to_imgmsg(cvim=frame, encoding='bgr8')
    image_msg.header.stamp = now
    
    rospy.loginfo("Camera is publishing a frame")
    pub.publish(image_msg)
    
    rate.sleep()