#!/usr/bin/env python3
import os
import sys
import signal
import fcntl
import time
import selectors

from docopt import docopt
import numpy as np

import donkeycar as dk

#import parts
from donkeycar.parts.transform import Lambda, TriggeredCallback, DelayedTrigger
from donkeycar.parts.datastore import TubHandler
from donkeycar.parts.controller import LocalWebController, JoystickController
from donkeycar.parts.throttle_filter import ThrottleFilter
from donkeycar.parts.behavior import BehaviorPart
from donkeycar.parts.file_watcher import FileWatcher
from donkeycar.parts.launch import AiLaunch
from donkeycar.utils import *

from donkeycar.parts.camera import PiCamera

class SSHReceiver:
    def __init__(self):
        orig_fl = fcntl.fcntl(sys.stdin, fcntl.F_GETFL)
        fcntl.fcntl(sys.stdin, fcntl.F_SETFL, orig_fl | os.O_NONBLOCK)
        self.auto_angle_delta = 0.00
        self.auto_speed_alpha = 1.00
        # set up stdin select for async IO
        self.sel = selectors.DefaultSelector()
        self.sel.register(sys.stdin, selectors.EVENT_READ, self.got_keyboard_data)
    def run_threaded(self):
        # run_threaded is called to return last instruction received
        return self.auto_angle_delta, self.auto_speed_alpha
    def update(self):
        # the update function runs in its own thread
        while True:
            for k, mask in self.sel.select():
                callback = k.data
                callback(k.fileobj)
    def got_keyboard_data(self, stdin):
        try:
            d = stdin.read()
        except EOFError:
            os.kill(os.getpid(), signal.SIGINT)
        d = d.split()
        if len(d) < 2:
            return
        print(d)
        self.auto_angle_delta = float(d[0])
        self.auto_speed_alpha = float(d[1])
