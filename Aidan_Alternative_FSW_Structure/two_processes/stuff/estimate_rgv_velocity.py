import rospy
from std_msgs.msg import Header
import collections


rgv_position_buffer = collections.deque([], 50)


def setup():
    def callback(msg):
        rgv_position_buffer.appendleft(msg)
        rospy.loginfo(f"'Estimate RGV Velocity' estimated a velocity from the most recent {len(rgv_position_buffer)} position predictions")
        pub.publish(msg)


    pub = rospy.Publisher("estimation/estimated_rgv_velocities", Header, queue_size=1)
    sub = rospy.Subscriber("estimation/estimated_rgv_locations", Header, callback)