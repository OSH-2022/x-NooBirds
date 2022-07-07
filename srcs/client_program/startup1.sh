#!/bin/bash

source /home/pi/donkeycar/car_env/bin/activate

# python /home/pi/mycar/stdin_drive.py drive --js
python /home/pi/mycar/stdin_drive_vel_factor.py drive --js
