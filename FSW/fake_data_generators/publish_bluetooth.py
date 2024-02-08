import rospy
from ..config.topic_names import RAW_BLUETOOTH
from rosardvarc.msg import BluetoothAzimuthElevation


rospy.init_node("bluetooth")
pub = rospy.Publisher(RAW_BLUETOOTH, BluetoothAzimuthElevation, queue_size=1)
rate = rospy.Rate(5)
while not rospy.is_shutdown():
    now = rospy.Time.now()
    pub.publish(
        BluetoothAzimuthElevation(
            # TODO: Make this something reasonable
        )
    )
    rate.sleep()