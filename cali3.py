import odrive
from odrive.enums import *
import time
import datetime
import csv
from datetime import date
from datetime import datetime

# 30-5 quick script for displaying shadowcount for calibration. 

global state
state = 0

global t_sleep, csv_writer
t_sleep = 0.05
csv_writer = None

# when already calibrated, set this to true to go straight into count display.
initialized = False

try: # Reboot causes loss of connection, use try to supress errors
    odrv0.reboot()
except:
    pass

odrv0 = odrive.find_any() # Reconnect to the Odrive

if not initialized:

    #odrv0.reboot()
    odrv0.clear_errors()
    

    odrv0.axis0.min_endstop.config.enabled = False
    odrv0.axis1.min_endstop.config.enabled = False

    odrv0.axis0.encoder.config.cpr = 8192
    odrv0.axis1.encoder.config.cpr = 8192

    odrv0.axis0.motor.config.pole_pairs = 11
    odrv0.axis1.motor.config.pole_pairs = 11

    odrv0.axis0.motor.config.resistance_calib_max_voltage = 2.0  # default is 1.0
    odrv0.axis1.motor.config.resistance_calib_max_voltage = 2.0  # default is 1.0

    #odrv0.save_configuration() for some reason save config line here disconnects it (27-5)

    #odrv0.axis0.config.startup_closed_loop_control = False
    #odrv0.axis1.config.startup_closed_loop_control = False



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
    odrv0.axis0.motor.config.current_lim = 50
    odrv0.axis0.controller.config.input_mode = INPUT_MODE_VEL_RAMP
    odrv0.axis0.controller.input_vel = 500
    
    while round(odrv0.axis0.motor.current_control.Iq_measured,2)<5: # Wait for calibration to be done
        #time.sleep(0.1)
        print("current: ",round(odrv0.axis0.motor.current_control.Iq_measured,2))
        
    odrv0.axis0.controller.input_vel = 0
    odrv0.axis0.encoder.set_linear_count(int(14.5*8192))
    odrv0.axis0.requested_state = AXIS_STATE_IDLE
    

    print("Motor 0: Homing complete")

    time.sleep(2)



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
    odrv0.axis1.motor.config.current_lim = 50
    odrv0.axis1.controller.config.input_mode = INPUT_MODE_VEL_RAMP
    odrv0.axis1.controller.input_vel = 500

    while round(odrv0.axis1.motor.current_control.Iq_measured,2)<5: # Wait for calibration to be done
        #time.sleep(0.1)
        print("current: ",round(odrv0.axis1.motor.current_control.Iq_measured,2))
    
    odrv0.axis1.controller.input_vel = 0
    odrv0.axis1.encoder.set_linear_count(int(14.5*8192))
    odrv0.axis1.requested_state = AXIS_STATE_IDLE

    time.sleep(2)


    print("Motor 1: Homing complete, waiting")



    initialized = True


print("move to middle")

    
print("moving...")


#'''
odrv0.axis0.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL
odrv0.axis1.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL

odrv0.axis0.controller.config.input_mode = INPUT_MODE_TRAP_TRAJ
odrv0.axis0.trap_traj.config.vel_limit = 10000
odrv0.axis0.trap_traj.config.accel_limit = 50
odrv0.axis0.trap_traj.config.decel_limit = 50

odrv0.axis1.controller.config.input_mode = INPUT_MODE_TRAP_TRAJ
odrv0.axis1.trap_traj.config.vel_limit = 10000
odrv0.axis1.trap_traj.config.accel_limit = 50
odrv0.axis1.trap_traj.config.decel_limit = 50

odrv0.axis0.controller.input_pos = 0
odrv0.axis1.controller.input_pos = 0

#odrv0.axis0.requested_state = AXIS_STATE_IDLE
#odrv0.axis1.requested_state = AXIS_STATE_IDLE
#'''



'''
while True:

    odrv0.axis0.requested_state = AXIS_STATE_IDLE
    odrv0.axis1.requested_state = AXIS_STATE_IDLE
    
    time.sleep(0.1)
    print("")
    print("")
    print("")
    print("")
    print("")
    print("Encoder 0 pos: ", odrv0.axis0.encoder.shadow_count/8192)
    print("Encoder 1 pos: ", odrv0.axis1.encoder.shadow_count/8192)
'''
    




