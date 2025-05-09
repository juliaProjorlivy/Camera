#!/bin/bash
set -x
sudo killall pigpiod
sudo pigpiod
/home/pi/env/bin/python3 /home/pi/camera/video_capture.py
