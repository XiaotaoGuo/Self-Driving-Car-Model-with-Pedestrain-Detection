#!/usr/bin/env python3
"""
Scripts to drive a donkey 2 car

Usage:
    manage.py (drive) [--model=<model>] [--js] [--type=(linear|categorical|rnn|imu|behavior|3d|localizer)] [--camera=(single|stereo)]
    manage.py (train) [--tub=<tub1,tub2,..tubn>] (--model=<model>) [--transfer=<model>] [--type=(linear|categorical|rnn|imu|behavior|3d|localizer)] [--continuous] [--aug]


Options:
    -h --help     Show this screen.
    --js          Use physical joystick.
"""
import os
import time
from docopt import docopt
import config as cfg

import donkeycar as dk

#import parts
from donkeycar.parts.transform import Lambda
from donkeycar.parts.datastore import TubHandler, TubGroup
from donkeycar.parts.controller import LocalWebController, JoystickController
from donkeycar.parts.imu import Mpu6050
import numpy as np
from donkeycar.parts.throttle_filter import ThrottleFilter
from donkeycar.parts.behavior import BehaviorPart
from donkeycar.parts.file_watcher import FileWatcher
from donkeycar.parts.camera import PiCamera


def test(cfg):
    V = dk.vehicle.Vehicle()
    inputs = []
    threaded = True

    #cam = PiCamera(image_w=cfg.IMAGE_W, image_h=cfg.IMAGE_H, image_d=cfg.IMAGE_DEPTH)
    cam = PiCamera(image_w=120, image_h=160, image_d=cfg.IMAGE_DEPTH)
    #cam = PiCamera(image_w=608, image_h=608, image_d=cfg.IMAGE_DEPTH)

    V.add(cam, inputs=inputs, outputs=['cam/image_array'], threaded=threaded)
    
    V.start(rate_hz=cfg.DRIVE_LOOP_HZ, max_loop_count=cfg.MAX_LOOPS)

if __name__ == '__main__':

    test(cfg)
    
