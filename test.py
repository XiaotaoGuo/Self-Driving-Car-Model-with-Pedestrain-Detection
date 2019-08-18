#!/usr/bin/env python3
"""
Scripts to drive a donkey 2 car

Usage:
    manage.py (drive) [--model=<model>] [--js] [--type=(linear|categorical|rnn|imu|behavior|3d|localizer|latent)] [--camera=(single|stereo)]
    manage.py (train) [--tub=<tub1,tub2,..tubn>] (--model=<model>) [--transfer=<model>] [--type=(linear|categorical|rnn|imu|behavior|3d|localizer)] [--continuous] [--aug]


Options:
    -h --help     Show this screen.
    --js          Use physical joystick.
"""
import os
import time
from docopt import docopt

import donkeycar as dk
from part_new import detector, drive_mode_select

#import parts
from donkeycar.parts.transform import Lambda
from donkeycar.parts.datastore import TubHandler, TubGroup
from donkeycar.parts.controller import LocalWebController, JoystickController
from donkeycar.parts.imu import Mpu6050
import numpy as np
from donkeycar.parts.throttle_filter import ThrottleFilter
from donkeycar.parts.behavior import BehaviorPart
from donkeycar.parts.file_watcher import FileWatcher

def drive(cfg, model_path=None, use_joystick=False, model_type=None, camera_type='single'):
    '''
    Construct a working robotic vehicle from many parts.
    Each part runs as a job in the Vehicle loop, calling either
    it's run or run_threaded method depending on the constructor flag `threaded`.
    All parts are updated one after another at the framerate given in
    cfg.DRIVE_LOOP_HZ assuming each part finishes processing in a timely manner.
    Parts may have named outputs and inputs. The framework handles passing named outputs
    to parts requesting the same named input.
    '''

    if model_type is None:
        if cfg.TRAIN_LOCALIZER:
            model_type = "localizer"
        elif cfg.TRAIN_BEHAVIORS:
            model_type = "behavior"
        else:
            model_type = "categorical"
    
    #Initialize car
    V = dk.vehicle.Vehicle()

    from donkeycar.parts.camera import PiCamera,Webcam
    cam = PiCamera(image_w=cfg.IMAGE_W, image_h=cfg.IMAGE_H, image_d=cfg.IMAGE_DEPTH)
    inputs = []
    threaded = True
    V.add(cam, inputs=inputs, outputs=['cam/image_array','detector/person'], threaded=threaded)
    
    #webcam = Webcam(image_w=cfg.IMAGE_W, image_h=cfg.IMAGE_H, image_d=cfg.IMAGE_DEPTH)
    #V.add(webcam,inputs=inputs,outputs=['detector/person'],threaded=threaded)
        
    #modify max_throttle closer to 1.0 to have more power
    #modify steering_scale lower than 1.0 to have less responsive steering
    #dt = detector()
    #V.add(dt,inputs=['cam/image_array'],outputs=['ctr/throttle','detector/person'],threaded = True)
    '''
    from donkeycar.parts.controller import PS3JoystickController, PS4JoystickController
    cont_class = PS3JoystickController

    ctr = cont_class(throttle_scale=cfg.JOYSTICK_MAX_THROTTLE,
                            steering_scale=cfg.JOYSTICK_STEERING_SCALE,
                            auto_record_on_throttle=cfg.AUTO_RECORD_ON_THROTTLE)
        
    ctr.set_deadzone(cfg.JOYSTICK_DEADZONE)
    
    V.add(ctr, 
          inputs=['cam/image_array'],
          outputs=['user/angle', 'user/throttle', 'user/mode', 'recording'],
          threaded=True)

    #this throttle filter will allow one tap back for esc reverse
    th_filter = ThrottleFilter()
    V.add(th_filter, inputs=['user/throttle'], outputs=['user/throttle'])
    '''
    #See if we should even run the pilot module. 
    #This is only needed because the part run_condition only accepts boolean
    '''
    def pilot_condition(mode):
        if mode == 'user':
            return False
        else:
            return True
        
    pilot_condition_part = Lambda(pilot_condition)
    V.add(pilot_condition_part, inputs=['user/mode'], outputs=['run_pilot'])

    def load_model(kl, model_path):
        start = time.time()
        try:
            print('loading model', model_path)
            kl.load(model_path)
            print('finished loading in %s sec.' % (str(time.time() - start)) )
        except Exception as e:
            print(e)
            print('ERR>> problems loading model', model_path)

    def load_weights(kl, weights_path):
        start = time.time()
        try:
            print('loading model weights', weights_path)
            kl.model.load_weights(weights_path)
            print('finished loading in %s sec.' % (str(time.time() - start)) )
        except Exception as e:
            print(e)
            print('ERR>> problems loading weights', weights_path)

    def load_model_json(kl, json_fnm):
        start = time.time()
        print('loading model json', json_fnm)
        import keras
        with open(json_fnm, 'r') as handle:
            contents = handle.read()
            kl.model = keras.models.model_from_json(contents)
        print('finished loading json in %s sec.' % (str(time.time() - start)) )

    if model_path:
        #When we have a model, first create an appropriate Keras part
        kl = dk.utils.get_model_by_type(model_type, cfg)

        if '.h5' in model_path:
            #when we have a .h5 extension
            #load everything from the model file
            load_model(kl, model_path)

            def reload_model(filename):
                print(filename, "was changed!")
                load_model(kl, filename)

            fw_part = FileWatcher(model_path, reload_model, wait_for_write_stop=10.0)
            V.add(fw_part, outputs=['reloaded/model'])

        elif '.json' in model_path:
            #when we have a .json extension
            #load the model from their and look for a matching
            #.wts file with just weights
            #load_model_json(kl, model_path)
            weights_path = model_path.replace('.json', '.weights')
            load_weights(kl, weights_path)

            def reload_weights(filename):
                print(filename, "was changed!")
                weights_path = filename.replace('.json', '.weights')
                load_weights(kl, weights_path)
            
            fw_part = FileWatcher(model_path, reload_weights, wait_for_write_stop=1.0)
            V.add(fw_part, outputs=['reloaded/model'])

        else:
            #previous default behavior
            load_model(kl, model_path)

        outputs=['pilot/angle', 'pilot/throttle']

        if cfg.TRAIN_LOCALIZER:
            outputs.append("pilot/loc")
    
        #V.add(kl, inputs=['cam/image_array'], outputs=outputs, run_condition='run_pilot')            
    '''
    #Choose what inputs should change the car.

    drv_mode = drive_mode_select()
    #V.add(drv_mode, inputs=['pilot/throttle','pilot/angle','detector/person'],outputs=['angle','throttle'])
    V.add(drv_mode, inputs=['detector/person'],outputs=['angle','throttle'])
    
    '''
    def drive_mode(mode, 
                   user_angle, user_throttle,
                   pilot_angle, pilot_throttle):
        if mode == 'user': 
            return user_angle, user_throttle
        
        elif mode == 'local_angle':
            return pilot_angle, user_throttle
        
        else: 
            return pilot_angle, pilot_throttle
        
    drive_mode_part = Lambda(drive_mode)

    V.add(drive_mode_part, 
          inputs=['user/mode', 'user/angle', 'user/throttle',
                  'pilot/angle', 'pilot/throttle'], 
          outputs=['angle', 'throttle'])
    '''

    #Drive train setup

    if cfg.DRIVE_TRAIN_TYPE == "SERVO_ESC":
        from donkeycar.parts.actuator import PCA9685, PWMSteering, PWMThrottle

        steering_controller = PCA9685(cfg.STEERING_CHANNEL, cfg.PCA9685_I2C_ADDR, busnum=cfg.PCA9685_I2C_BUSNUM)
        steering = PWMSteering(controller=steering_controller,
                                        left_pulse=cfg.STEERING_LEFT_PWM, 
                                        right_pulse=cfg.STEERING_RIGHT_PWM)
        
        throttle_controller = PCA9685(cfg.THROTTLE_CHANNEL, cfg.PCA9685_I2C_ADDR, busnum=cfg.PCA9685_I2C_BUSNUM)
        throttle = PWMThrottle(controller=throttle_controller,
                                        max_pulse=cfg.THROTTLE_FORWARD_PWM,
                                        zero_pulse=cfg.THROTTLE_STOPPED_PWM, 
                                        min_pulse=cfg.THROTTLE_REVERSE_PWM)

        V.add(steering, inputs=['angle'])
        V.add(throttle, inputs=['throttle'])
    '''
    if type(ctr) is LocalWebController:
        print("You can now go to <your pi ip address>:8887 to drive your car.")
    elif isinstance(ctr, JoystickController):
        print("You can now move your joystick to drive your car.")
        #tell the controller about the tub        
        ctr.set_tub(tub)
        if cfg.BUTTON_PRESS_NEW_TUB:
    
            def new_tub_dir():
                V.parts.pop()
                tub = th.new_tub_writer(inputs=inputs, types=types)
                V.add(tub, inputs=inputs, outputs=["tub/num_records"], run_condition='recording')
                ctr.set_tub(tub)
    
            ctr.set_button_down_trigger('cross', new_tub_dir)
    '''
    #run the vehicle for 20 seconds
    V.start(rate_hz=cfg.DRIVE_LOOP_HZ, 
            max_loop_count=cfg.MAX_LOOPS)


if __name__ == '__main__':
    args = docopt(__doc__)
    cfg = dk.load_config()
    
    if args['drive']:
        model_type = args['--type']
        camera_type = args['--camera']
        drive(cfg, model_path = args['--model'], use_joystick=args['--js'], model_type=model_type, camera_type=camera_type)
    
    if args['train']:
        print('only use drive mode!')



