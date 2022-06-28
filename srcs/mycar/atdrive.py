#!/usr/bin/env python3
"""
Scripts to drive a donkey 2 car

Usage:
    manage.py (drive) [--model=<model>] [--js] [--type=(linear|categorical|rnn|imu|behavior|3d|localizer|latent)] [--camera=(single|stereo)] [--meta=<key:value> ...]
    manage.py (train) [--tub=<tub1,tub2,..tubn>] [--file=<file> ...] (--model=<model>) [--transfer=<model>] [--type=(linear|categorical|rnn|imu|behavior|3d|localizer)] [--continuous] [--aug]


Options:
    -h --help          Show this screen.
    --js               Use physical joystick.
    -f --file=<file>   A text file containing paths to tub files, one per line. Option may be used more than once.
    --meta=<key:value> Key/Value strings describing describing a piece of meta data about this drive. Option may be used more than once.
"""
import os
import time

from docopt import docopt
import numpy as np

import donkeycar as dk

#import parts
from donkeycar.parts.transform import Lambda, TriggeredCallback, DelayedTrigger
from donkeycar.parts.datastore import TubHandler
from donkeycar.parts.controller import LocalWebController, JoystickController
import CrappyJoystick
from donkeycar.parts.throttle_filter import ThrottleFilter
from donkeycar.parts.behavior import BehaviorPart
from donkeycar.parts.file_watcher import FileWatcher
from donkeycar.parts.launch import AiLaunch
from donkeycar.utils import *

from donkeycar.parts.camera import PiCamera

def truncfloat(x, absv):
    absv = abs(absv)
    if x > absv:
        return absv
    if x < -absv:
        return -absv
    return x

args = docopt(__doc__)
cfg = dk.load_config()

V = dk.vehicle.Vehicle()

import cv2
import math
class LineFollower:
    def __init__(self, debug=False):
        self.debug = debug
        self.pdirection = 0.0
    def run(self, image):
        # image shape is (120, 160, 3)
        if not image is None:
            if self.debug:
                # bull race test
                avg = 0.00
                cnt = 0
                for x in range(160):
                    for y in range(120):
                        if image[y,x,0] < 20 and image[y,x,1] < 20 and image[y,x,2] < 20:
                            avg += x
                            cnt += 1
                if cnt == 0:
                    return 0.0
                avg /= cnt
                steer = - (avg - 80) / 80
                print('steer =', steer)
                return steer
            else:
                # detect color
                image = image[50:, :, :]
                
                gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
                
                retval, dst = cv2.threshold(gray, 0, 255, cv2.THRESH_OTSU)
                
                dst = cv2.erode(dst, None, iterations=6)
                
                color = dst[50]
                try:
                    black_count = np.sum(color <= 20)
                    black_index = np.where(color <= 20)
                    print(black_count)
                    if black_count <= 2:
                        black_count = 0
                    center = (black_index[0][black_count-1] + black_index[0][0]) / 2
                    direction = (center - 80) / 80
                    self.pdirection = self.pdirection + (direction - self.pdirection) * 1.5
                    if self.pdirection >= 1:
                        self.pdirection = 1.0
                    elif self.pdirection <= -1.0:
                        self.pdirection = -1.0
                    return -self.pdirection
                except:
                    return -self.pdirection

        return 0.0

cam = PiCamera(image_w=cfg.IMAGE_W, image_h=cfg.IMAGE_H, image_d=cfg.IMAGE_DEPTH)
V.add(cam, inputs=[], outputs=['cam/image_array'], threaded=True)

V.add(LineFollower(), inputs=['cam/image_array'], outputs=['pilot/angle'])


from donkeycar.parts.controller import get_js_controller
ctr = CrappyJoystick.CrappyJoystickController()
V.add(ctr,
      inputs=['cam/image_array'],
      outputs=['user/angle', 'user/throttle', 'user/mode', 'recording'],
      threaded=True)


class DriveMode:
    def run(self, mode,
            user_angle, user_throttle,
            pilot_angle, pilot_throttle):
        if user_angle == 0.00:
            mode='local_angle'
        # print(mode, user_angle, user_throttle)
        if mode == 'user':
            return user_angle, user_throttle
        elif mode == 'local_angle':
            return pilot_angle, user_throttle
        else:
            # AI_THROTTLE_MULT is potentially helpful for velocity control
            return pilot_angle, pilot_throttle * cfg.AI_THROTTLE_MULT

V.add(DriveMode(),
        inputs=['user/mode', 'user/angle', 'user/throttle',
                'pilot/angle', 'pilot/throttle'],
        outputs=['angle', 'throttle'])


# cfg.DRIVE_TRAIN_TYPE == "SERVO_ESC":
from donkeycar.parts.actuator import PCA9685, PWMSteering, PWMThrottle
steering_controller = PCA9685(cfg.STEERING_CHANNEL, cfg.PCA9685_I2C_ADDR, busnum=cfg.PCA9685_I2C_BUSNUM)
steering = PWMSteering(controller=steering_controller,
                                left_pulse=cfg.STEERING_LEFT_PWM, 
                                right_pulse=cfg.STEERING_RIGHT_PWM)
throttle_controller = PCA9685(cfg.THROTTLE_CHANNEL, cfg.PCA9685_I2C_ADDR1, frequency=1600, busnum=cfg.PCA9685_I2C_BUSNUM)
throttle = PWMThrottle(controller=throttle_controller,
                                max_pulse=cfg.THROTTLE_FORWARD_PWM,
                                zero_pulse=cfg.THROTTLE_STOPPED_PWM, 
                                min_pulse=cfg.THROTTLE_REVERSE_PWM)
# normalized data from input
V.add(steering, inputs=['angle'])
V.add(throttle, inputs=['throttle'])



#warmup camera
while cam.run() is None:
    time.sleep(1)

ctr.print_controls()

V.start(rate_hz=30, max_loop_count=cfg.MAX_LOOPS)
