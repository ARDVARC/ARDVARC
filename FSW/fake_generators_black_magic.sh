#!/bin/bash

cd ..
roscore &
python3 -m FSW.cv.py &
python3 -m FSW.main.py &
python3 -m FSW.fake_data_generators.fake_mavros.py &
python3 -m FSW.fake_data_generators.publish_camera.py &
python3 -m FSW.fake_data_generators.publish_bluetooth.py