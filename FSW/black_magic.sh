#!/bin/bash

roscore &
python3 cv.py &
python3 main.py
