import time
import rospy
from std_msgs.msg import Header, Time


def callback(msg):
    # Skip this frame if it is too old
    temp: Time = msg.stamp
    if (rospy.Time.now() - temp).to_sec() > 0.1:
        rospy.loginfo(f"Skipping frame #{msg.seq}")
        return
    
    now = time.time()
    
    # Do 200 ms worth of junk
    x = 0
    while (time.time() - now) < 0.2:
        x = 123.45 ** 67 + x / 8 ** 90
    
    rospy.loginfo(f"Prcoess frame finished processing frame #{msg.seq} at {time.time()}")
    annotated_frame_pub.publish(msg)
    rgv_sightings_pub.publish(msg)
    direction_vector_pub.publish(msg)


rospy.init_node("process_frame")
annotated_frame_pub = rospy.Publisher("camera/annotated_frames", Header, queue_size=1)
rgv_sightings_pub = rospy.Publisher("camera/rgv_sightings", Header, queue_size=1)
direction_vector_pub = rospy.Publisher("estimation/direction_vectors_uas", Header, queue_size=1)
sub = rospy.Subscriber("camera/frames", Header, callback)

while not rospy.is_shutdown():
    rospy.spin()