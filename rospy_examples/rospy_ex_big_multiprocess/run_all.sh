#!/bin/bash

# Starts roscore along with all 5 processes
roscore &
python3 node_1.py &
python3 node_2.py &
python3 node_3.py &
python3 node_4.py &
python3 node_5.py