import rospy
from std_msgs.msg import Header


rospy.init_node("camera")
pub = rospy.Publisher("camera/frames", Header, queue_size=1)
rate = rospy.Rate(60)
frame_count = 1
while not rospy.is_shutdown():
    now = rospy.Time.now()
    pub.publish(
        Header(
            seq = frame_count,
            stamp = now,
            frame_id=""
        )
    )
    rospy.loginfo(f"Camera published frame #{frame_count} at t={now}")
    
    frame_count += 1
    rate.sleep()