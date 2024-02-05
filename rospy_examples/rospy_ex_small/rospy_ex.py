import time
import rospy
import genpy
from rosardvarc.msg import BluetoothAzimuthElevation
import math


def print_callback(message):
    print("Got a callback!:")
    print(message)


rospy.init_node("rospy_node")
pub = rospy.Publisher("az_el_topic", BluetoothAzimuthElevation, queue_size=1)
sub = rospy.Subscriber("az_el_topic", BluetoothAzimuthElevation, print_callback)
rate = rospy.Rate(1)
while not rospy.is_shutdown():
    now = time.time()
    now_sec = int(math.floor(now))
    now_nano = int((now-math.floor(now))*1e9)
    pub.publish(
        BluetoothAzimuthElevation(
            timestamp=genpy.Time(now_sec, now_nano),
            rgv_id=BluetoothAzimuthElevation.RGV_1,
            azimuth=123.45,
            elevation=67.89
        )
    )
    rate.sleep()