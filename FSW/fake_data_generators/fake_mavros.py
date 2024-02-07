import rospy
from std_msgs.msg import Header


def setpoints_callback(msg):
    rospy.loginfo("MAVROS received a setpoint, sending to pixhawk")


def roi_callback(msg):
    rospy.loginfo("MAVROS received an ROI, sending to pixhawk")


rospy.init_node("mavros")
raw_gps_pub = rospy.Publisher("pixhawk/raw_gps_data", Header, queue_size=1)
uas_state_pub = rospy.Publisher("pixhawk/uas_states", Header, queue_size=1)
setpoints_sub = rospy.Subscriber("pixhawk/setpoints", Header, setpoints_callback)
roi_sub = rospy.Subscriber("pixhawk/rois", Header, roi_callback)
rate = rospy.Rate(20)
while not rospy.is_shutdown():
    now = rospy.Time.now()
    raw_gps_pub.publish(
        Header(
            seq = 0,
            stamp = now,
            frame_id=""
        )
    )
    uas_state_pub.publish(
        Header(
            seq = 0,
            stamp = now,
            frame_id=""
        )
    )
    # rospy.loginfo(f"MAVROS published some stuff at t={now}")
    rate.sleep()