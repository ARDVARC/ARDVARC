#!/bin/bash

roscore &
python3 cv.py &
python3 main.py &
python3 fake_mavros.py &
python3 publish_camera.py &
python3 publish_bluetooth.py