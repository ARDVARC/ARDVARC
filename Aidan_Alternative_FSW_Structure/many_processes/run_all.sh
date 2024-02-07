#!/bin/bash

roscore &
python3 determine_mission_state.py &
python3 determine_setpoint_and_roi.py &
python3 estimate_rgv_state.py &
python3 estimate_rgv_velocity.py &
python3 fake_mavros.py &
python3 generate_state_machine_criteria.py &
python3 process_bluetooth.py &
python3 process_frame.py &
python3 publish_camera.py &
python3 publish_bluetooth.py