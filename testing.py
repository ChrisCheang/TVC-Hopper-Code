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

if initialized:

    odrv0 = odrive.find_any() # Reconnect to the Odrive

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

    print("starting motor 1 axis state homing")
    odrv0.axis1.requested_state = AXIS_STATE_HOMING
    while odrv0.axis1.current_state != AXIS_STATE_IDLE: # Wait for calibration to be done
        time.sleep(0.1)
        print(".", end="")

    print("Motor 1: Homing complete, waiting [9/9]")

    
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

    print("starting motor 0 axis state homing")
    odrv0.axis0.requested_state = AXIS_STATE_HOMING
    while odrv0.axis0.current_state != AXIS_STATE_IDLE: # Wait for calibration to be done
        time.sleep(0.1)
        print(".", end="")

    print("Motor 0: Homing complete [8/9]")

    

    initialized = True


while True:
    odrv0 = odrive.find_any() # Reconnect to the Odrive
    print(odrv0.get_adc_voltage(3))
    time.sleep(0.1)



