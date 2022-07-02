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
import sys
import signal

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

import receiver

# handle SIGHUP from remote ssh
def sighup_handler(sig, frame):
    os.kill(os.getpid(), signal.SIGINT)

signal.signal(signal.SIGHUP, sighup_handler)

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
    def run(self, image):
        return 0.0

# cam = PiCamera(image_w=cfg.IMAGE_W, image_h=cfg.IMAGE_H, image_d=cfg.IMAGE_DEPTH)
# V.add(cam, inputs=[], outputs=['cam/image_array'], threaded=True)

# V.add(LineFollower(), inputs=['cam/image_array'], outputs=['pilot/angle'])
V.add(receiver.SSHReceiver(), inputs=[], outputs=['pilot/angle', 'pilot/throttle'], threaded=True)


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
        mode='pilot'
        # print(mode, user_angle, user_throttle)
        if mode == 'user':
            return user_angle, user_throttle
        elif mode == 'local_angle':
            return pilot_angle, user_throttle
        elif mode == 'pilot':
            return pilot_angle, pilot_throttle
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
# while cam.run() is None:
#     time.sleep(1)

ctr.print_controls()

V.start(rate_hz=30, max_loop_count=cfg.MAX_LOOPS)
