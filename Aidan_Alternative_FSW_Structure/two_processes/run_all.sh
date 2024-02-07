#!/bin/bash

roscore &
python3 everything_else.py &
python3 fake_mavros.py &
python3 process_frame.py &
python3 publish_camera.py &
python3 publish_bluetooth.py