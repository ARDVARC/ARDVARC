import rospy
from std_msgs.msg import Header
import collections


uas_state_buffer = collections.deque([], 50)
direction_vector_buffer = collections.deque([], 50)


def uas_state_callback(msg):
    uas_state_buffer.appendleft(msg)


def direction_vector_callback(msg):
    direction_vector_buffer.appendleft(msg)


rospy.init_node("estimate_rgv_state")
pub = rospy.Publisher("estimation/estimated_rgv_locations", Header, queue_size=1)
uas_state_sub = rospy.Subscriber("pixhawk/uas_states", Header, uas_state_callback)
direction_vector_sub = rospy.Subscriber("estimation/direction_vectors_uas", Header, direction_vector_callback)
estimate_count = 1
rate = rospy.Rate(3)
now = rospy.Time.now()
while not rospy.is_shutdown():
    pub.publish(
        Header(
            seq = estimate_count,
            stamp = now,
            frame_id=""
        )
    )
    rospy.loginfo(f"'Estimate RGV State' published estimate #{estimate_count} at t={now} based on the most recent {len(uas_state_buffer)} UAS state measurements and the most recent {len(direction_vector_buffer)} RGV measurements")
    estimate_count += 1
    rate.sleep()