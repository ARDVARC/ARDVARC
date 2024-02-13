import rospy
from ..config.topic_names import UAS_POSES, SETPOINTS, REGIONS_OF_INTEREST
from geometry_msgs.msg import PoseStamped
from rosardvarc.msg import Setpoint, RegionOfInterest


def setpoints_callback(msg: Setpoint):
    rospy.loginfo("MAVROS received a setpoint, sending to pixhawk")
    pass


def roi_callback(msg: RegionOfInterest):
    rospy.loginfo("MAVROS received an ROI, sending to pixhawk")
    pass


rospy.init_node("fake_mavros")
uas_state_pub = rospy.Publisher(UAS_POSES, PoseStamped, queue_size=1)
setpoints_sub = rospy.Subscriber(SETPOINTS, Setpoint, setpoints_callback)
roi_sub = rospy.Subscriber(REGIONS_OF_INTEREST, RegionOfInterest, roi_callback)
rate = rospy.Rate(20)
while not rospy.is_shutdown():
    now = rospy.Time.now()
    rospy.loginfo("MAVROS is publishing a UAS pose")
    uas_state_msg = PoseStamped(
        # TODO: Make this something reasonable
    )
    uas_state_msg.header.stamp = now
    uas_state_pub.publish(uas_state_msg)
    rate.sleep()