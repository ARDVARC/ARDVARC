import time
import rospy
from std_msgs.msg import Header


def setup():
    def callback(msg):
        # Skip this bluetooth measurement if it is too old
        if (rospy.Time.now() - msg.stamp).to_sec() > 0.1:
            rospy.loginfo(f"Skipping bluetooth measurement #{msg.seq}")
            return
        
        now = time.time()
        
        # Do 1 ms worth of junk
        x = 0
        while (time.time() - now) < 0.001:
            x = 123.45 ** 67 + x / 8 ** 90
        
        rospy.loginfo(f"Prcoess bluetooth finished processing bluetooth measurement #{msg.seq} at {time.time()}")
        pub.publish(msg)


    pub = rospy.Publisher("estimation/direction_vectors_uas", Header, queue_size=1)
    sub = rospy.Subscriber("bluetooth/az_els", Header, callback)