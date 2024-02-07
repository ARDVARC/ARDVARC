import rospy
from std_msgs.msg import Header
from stuff import determine_mission_state, determine_setpoint_and_roi, estimate_rgv_state, estimate_rgv_velocity, generate_state_machine_criteria, process_bluetooth


rospy.init_node("everything_else")
determine_mission_state.setup()
determine_setpoint_and_roi.setup()
pub = estimate_rgv_state.setup()
estimate_rgv_velocity.setup()
generate_state_machine_criteria.setup()
process_bluetooth.setup()


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
    rospy.loginfo(f"'Estimate RGV State' published estimate #{estimate_count} at t={now} based on the most recent {len(estimate_rgv_state.uas_state_buffer)} UAS state measurements and the most recent {len(estimate_rgv_state.direction_vector_buffer)} RGV measurements")
    estimate_count += 1
    rate.sleep()