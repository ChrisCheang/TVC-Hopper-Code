import odrive
from odrive.enums import *
import time
import datetime
import csv
from datetime import date
from datetime import datetime

#from fibre.libfibre import ObjectLostError

global state
state = 0

global t_sleep, csv_writer
t_sleep = 0.05
csv_writer = None

initialized = False


odrv0 = odrive.find_any() # Reconnect to the Odrive


if not initialized:

    odrv0.axis0.min_endstop.config.enabled = False
    odrv0.axis1.min_endstop.config.enabled = False

    odrv0.axis0.encoder.config.cpr = 8192
    odrv0.axis1.encoder.config.cpr = 8192

    odrv0.axis0.motor.config.pole_pairs = 11
    odrv0.axis1.motor.config.pole_pairs = 11

    odrv0.save_configuration()
    
    print("starting motor 0 axis state motor calibration")
    odrv0.axis0.requested_state = AXIS_STATE_FULL_CALIBRATION_SEQUENCE  #AXIS_STATE_MOTOR_CALIBRATION (see if this is the problem)
    while odrv0.axis0.current_state != AXIS_STATE_IDLE: # Wait for calibration to be done
        time.sleep(0.1)
        print(".", end="")

    print("starting motor 0 axis state encoder offset calibration")
    odrv0.axis0.requested_state = AXIS_STATE_ENCODER_OFFSET_CALIBRATION
    while odrv0.axis0.current_state != AXIS_STATE_IDLE: # Wait for calibration to be done
        time.sleep(0.1)
        print(".", end="")

    print("starting motor 0 axis state homing (current limit)")
    odrv0.axis0.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL
    odrv0.axis0.controller.config.vel_ramp_rate = 0.5
    odrv0.axis0.motor.config.current_lim = 30
    odrv0.axis0.controller.config.input_mode = INPUT_MODE_VEL_RAMP
    odrv0.axis0.controller.input_vel = 500
    
    while round(odrv0.axis0.motor.current_control.Iq_measured,2)<3: # Wait for calibration to be done
        #time.sleep(0.1)
        print("current: ",round(odrv0.axis0.motor.current_control.Iq_measured,2))
        
    odrv0.axis0.controller.input_vel = 0
    odrv0.axis0.requested_state = AXIS_STATE_IDLE
    odrv0.axis0.encoder.set_linear_count(0)



    print("Motor 0: Homing complete")

    print("starting motor 1 axis state motor calibration")
    odrv0.axis1.requested_state = AXIS_STATE_FULL_CALIBRATION_SEQUENCE  #AXIS_STATE_MOTOR_CALIBRATION (see if this is the problem)
    while odrv0.axis1.current_state != AXIS_STATE_IDLE: # Wait for calibration to be done
        time.sleep(0.1)
        print(".", end="")

    print("starting motor 1 axis state encoder offset calibration")
    odrv0.axis1.requested_state = AXIS_STATE_ENCODER_OFFSET_CALIBRATION
    while odrv0.axis1.current_state != AXIS_STATE_IDLE: # Wait for calibration to be done
        time.sleep(0.1)
        print(".", end="")

    print("starting motor 1 axis state homing (current limit)") # endstops not working, 21-5
    odrv0.axis1.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL
    odrv0.axis1.controller.config.vel_ramp_rate = 0.5
    odrv0.axis1.motor.config.current_lim = 30
    odrv0.axis1.controller.config.input_mode = INPUT_MODE_VEL_RAMP
    odrv0.axis1.controller.input_vel = 500

    while round(odrv0.axis1.motor.current_control.Iq_measured,2)<3: # Wait for calibration to be done
        #time.sleep(0.1)
        print("current: ",round(odrv0.axis1.motor.current_control.Iq_measured,2))
    
    odrv0.axis1.controller.input_vel = 0
    odrv0.axis1.encoder.set_linear_count(0)
    odrv0.axis1.requested_state = AXIS_STATE_IDLE



    print("Motor 1: Homing complete, waiting")



    initialized = True

print("move to middle")

while True:
    
    print("moving...")

    odrv0.axis0.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL
    
    odrv0.axis0.controller.config.input_mode = INPUT_MODE_TRAP_TRAJ
    odrv0.axis0.trap_traj.config.vel_limit = 10
    odrv0.axis0.trap_traj.config.accel_limit = 10
    odrv0.axis0.trap_traj.config.decel_limit = 10
    odrv0.axis0.controller.input_pos = -5

    odrv0.axis1.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL

    odrv0.axis1.controller.config.input_mode = INPUT_MODE_TRAP_TRAJ
    odrv0.axis1.trap_traj.config.vel_limit = 10
    odrv0.axis1.trap_traj.config.accel_limit = 10
    odrv0.axis1.trap_traj.config.decel_limit = 10
    odrv0.axis1.controller.input_pos = -5



    
    # Endstop Testing (21-5)
    '''
    odrv0 = odrive.find_any() # Reconnect to the Odrive
    #print("endstop 0 = ", odrv0.axis0.min_endstop.endstop_state)
    #print("endstop 1 = ", odrv0.axis1.min_endstop.endstop_state)
    gpio_states = odrv0.get_gpio_states()
    gpio5_state = (gpio_states >> 5) & 1
    print(f"GPIO5 state: {'HIGH' if gpio5_state else 'LOW'}")
    #print("GPIO states = ", odrv0.get_gpio_states())
    time.sleep(0.05)
    '''




