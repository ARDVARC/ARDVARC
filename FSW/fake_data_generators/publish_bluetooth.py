import rospy
from std_msgs.msg import Header


rospy.init_node("bluetooth")
pub = rospy.Publisher("bluetooth/az_els", Header, queue_size=1)
rate = rospy.Rate(5)
bt_count = 1
while not rospy.is_shutdown():
    now = rospy.Time.now()
    pub.publish(
        Header(
            seq = bt_count,
            stamp = now,
            frame_id=""
        )
    )
    # rospy.loginfo(f"Bluetooth published bluetooth measurement #{bt_count} at t={now}")
    bt_count += 1
    rate.sleep()