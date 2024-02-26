#!/bin/bash

roscore &
python3 -m FSW.cv.py &
python3 -m FSW.main.py
