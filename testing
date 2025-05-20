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

def idle_state(odrv0):
    print("entered idle")
    global state

    odrv0.axis0.requested_state = AXIS_STATE_IDLE
    odrv0.axis1.requested_state = AXIS_STATE_IDLE

    print("idle")
    while state == 0:
        check_state(odrv0)
        time.sleep(t_sleep)



def full_reset_and_calibrate(odrv0):
    print("Entered Full Reset and Calibration")
    global state
    #Completely resets the Odrive, calibrates axis0 and configures axis0 to only encoder index search on startup and be ready in AXIS_STATE_CLOSED_LOOP_CONTROL
    try: # Reboot causes loss of connection, use try to supress errors
        odrv0.erase_configuration()
    except:
        pass
    odrv0 = odrive.find_any() # Reconnect to the Odrive
    print("Odrive: Erased [1/9]")

    try: # Reboot causes loss of connection, use try to supress errors
        odrv0.reboot()
    except:
        pass
    print("Odrive: Rebooted [2/9]")
    odrv0 = odrive.find_any() # Reconnect to the Odrive
    print("Odrive: Connected [3/9]")


    odrv0.config.enable_brake_resistor = True
    odrv0.config.brake_resistance = 5
    odrv0.config.dc_max_negative_current = -10
    odrv0.config.gpio5_mode = GPIO_MODE_DIGITAL
    # odrv0.config.gpio5_mode = GPIO_MODE_DIGITAL_PULL_UP
    odrv0.config.gpio4_mode = GPIO_MODE_DIGITAL
    # odrv0.config.gpio4_mode = GPIO_MODE_DIGITAL_PULL_UP
    print("Odrive: Parameters set [4/9]")


    #motor setup
    odrv0.axis0.motor.config.current_lim = 10
    odrv0.axis0.controller.config.vel_limit = 100
    odrv0.axis0.motor.config.pole_pairs = 7
    odrv0.axis0.motor.config.torque_constant = 0.05907142857
    odrv0.axis0.encoder.config.calib_scan_distance = 20
    odrv0.axis0.encoder.config.cpr = 8192
    #GPIO setup
    odrv0.axis0.min_endstop.config.gpio_num = 5
    odrv0.axis0.min_endstop.config.is_active_high = False
    # odrv0.axis0.min_endstop.config.offset = -10.5
    odrv0.axis0.min_endstop.config.offset = -10.3  #-10.5
    odrv0.axis0.min_endstop.config.enabled = True

    #homing setup
    odrv0.axis0.controller.config.homing_speed = -2
    odrv0.axis0.controller.config.vel_ramp_rate = 0.5
    odrv0.axis0.trap_traj.config.vel_limit = 2
    odrv0.axis0.trap_traj.config.accel_limit = 2
    odrv0.axis0.trap_traj.config.decel_limit = 2

    print("Motor 0: Calibration parameters set [5/9]")

    #motor setup
    odrv0.axis1.motor.config.current_lim = 10
    odrv0.axis1.controller.config.vel_limit = 100
    odrv0.axis1.motor.config.pole_pairs = 7
    odrv0.axis1.motor.config.torque_constant = 0.05907142857
    odrv0.axis1.encoder.config.calib_scan_distance = 20
    odrv0.axis1.encoder.config.cpr = 8192
    #endstop setup
    odrv0.axis1.min_endstop.config.gpio_num = 4
    odrv0.axis1.min_endstop.config.is_active_high = False
    # odrv0.axis1.min_endstop.config.offset = -10.5
    odrv0.axis1.min_endstop.config.offset = -10.3  #-10.5
    odrv0.axis1.min_endstop.config.enabled = True

    #homing setup
    odrv0.axis1.controller.config.homing_speed = -2
    odrv0.axis1.controller.config.vel_ramp_rate = 0.5
    odrv0.axis1.trap_traj.config.vel_limit = 2
    odrv0.axis1.trap_traj.config.accel_limit = 2
    odrv0.axis1.trap_traj.config.decel_limit = 2



    print("Motor 1: Calibration parameters set [6/9]")


    try: # Reboot causes loss of connection, use try to supress errors
        odrv0.save_configuration()
    except:
        pass
    odrv0 = odrive.find_any() # Reconnect to the Odrive

    print("Odrive: Config Saved [7/9]")

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

    while True:
        check_state(odrv0)
        print("debug state: ", state)
        if state != 1:

            state_machine(odrv0)
        time.sleep(t_sleep)




def lock_pos(odrv0,a,b):
    print("entered lock pos")
    #odrv0.axis0.requested_state = AXIS_STATE_IDLE
    #time.sleep(0.1)
    odrv0.axis0.motor.config.current_lim = 30

    odrv0.axis0.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL
    odrv0.axis0.controller.config.input_mode = INPUT_MODE_TRAP_TRAJ
    odrv0.axis0.trap_traj.config.vel_limit = 20
    odrv0.axis0.trap_traj.config.accel_limit = 20
    odrv0.axis0.trap_traj.config.decel_limit = 20

    #odrv0.axis1.requested_state = AXIS_STATE_IDLE
    #time.sleep(0.1)
    odrv0.axis1.motor.config.current_lim = 30

    odrv0.axis1.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL
    odrv0.axis1.controller.config.input_mode = INPUT_MODE_TRAP_TRAJ
    odrv0.axis1.trap_traj.config.vel_limit = 20
    odrv0.axis1.trap_traj.config.accel_limit = 20
    odrv0.axis1.trap_traj.config.decel_limit = 20

    odrv0.axis0.controller.input_pos = (odrv0.axis0.encoder.shadow_count/8192)
    odrv0.axis1.controller.input_pos = (odrv0.axis1.encoder.shadow_count/8192)
    #odrv0.axis0.controller.input_pos = 0
    #odrv0.axis1.controller.input_pos = 0

    while True:
        check_state(odrv0)
        if state != 2:
            state_machine(odrv0)
        time.sleep(t_sleep)


def test_procedure(odrv0):
    print("entered test procedure")
    START_POS_R2 = 0
    START_POS_D2 = 0
    CPR = 8192

    odrv0.axis0.motor.config.current_lim = 30
    odrv0.axis0.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL
    odrv0.axis0.controller.config.input_filter_bandwidth = 3 # Set the filter bandwidth [1/s]
    odrv0.axis0.controller.config.input_mode = INPUT_MODE_POS_FILTER # Activate the setpoint filter
    
    odrv0.axis1.motor.config.current_lim = 30
    odrv0.axis1.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL
    odrv0.axis1.controller.config.input_filter_bandwidth = 3 # Set the filter bandwidth [1/s]
    odrv0.axis1.controller.config.input_mode = INPUT_MODE_POS_FILTER # Activate the setpoint filter


    #move to 14 up
    odrv0.axis0.controller.input_pos=7.1774
    odrv0.axis1.controller.input_pos=7.1774
    time.sleep(1.2)

    odrv0.axis0.controller.config.input_filter_bandwidth = 9 # Increase filter bandwidth for circles
    odrv0.axis1.controller.config.input_filter_bandwidth = 9 # Increase filter bandwidth for circles


    #14 deg circle - starts at 14 up, circle, ends at 14 up (65 waypoints)
    waypoints0 = [-7.1774,-6.4577,-5.6811,-4.8557,-3.9901,-3.0931,-2.1735,-1.2403,-0.3026,0.6309,1.5512,2.4500,3.3189,4.1500,4.9358,5.6692,6.3436,6.9531,7.4922,7.9561,8.3410,8.6434,8.8608,8.9915,9.0344,8.9893,8.8568,8.6380,8.3350,7.9505,7.4876,6.9505,6.3436,5.6722,4.9421,4.1595,3.3312,2.4646,1.5674,0.6478,-0.2858,-1.2245,-2.1594,-3.0811,-3.9807,-4.8489,-5.6768,-6.4558,-7.1774,-7.8341,-8.4186,-8.9245,-9.3462,-9.6791,-9.9194,-10.0645,-10.1127,-10.0635,-9.9176,-9.6766,-9.3433,-8.9216,-8.4161,-7.8326,-7.1774]
    waypoints1 = [-7.1774,-7.8326,-8.4161,-8.9216,-9.3433,-9.6766,-9.9176,-10.0635,-10.1127,-10.0645,-9.9194,-9.6791,-9.3462,-8.9245,-8.4185,-7.8341,-7.1774,-6.4557,-5.6768,-4.8489,-3.9807,-3.0811,-2.1593,-1.2245,-0.2858,0.6478,1.5674,2.4646,3.3312,4.1595,4.9421,5.6723,6.3436,6.9505,7.4876,7.9505,8.3350,8.6380,8.8568,8.9893,9.0344,8.9915,8.8608,8.6434,8.3410,7.9561,7.4922,6.9531,6.3436,5.6692,4.9358,4.1500,3.3189,2.4500,1.5512,0.6308,-0.3026,-1.2403,-2.1735,-3.0931,-3.9902,-4.8557,-5.6811,-6.4577,-7.1774]

    waypoints = [[-waypoints0[i],-waypoints1[i]] for i in range(len(waypoints0))]

    for i in waypoints:
        odrv0.axis0.controller.input_pos=i[0]
        odrv0.axis1.controller.input_pos=i[1]

        check_state(odrv0)
        if state != 3:
            state_machine(odrv0)
        time.sleep(0.03)

    #move to 10 deg up to prepare for fast circle
    odrv0.axis0.controller.config.input_filter_bandwidth = 8 # Decrease filter bandwidth to move to 10 deg up
    odrv0.axis1.controller.config.input_filter_bandwidth = 8 # Decrease filter bandwidth to move to 10 deg up
    odrv0.axis0.controller.input_pos = 5.06  #10 deg up
    odrv0.axis1.controller.input_pos = 5.06
    time.sleep(0.8)


    odrv0.axis0.controller.config.input_filter_bandwidth = 50 # Increase filter bandwidth for circles
    odrv0.axis1.controller.config.input_filter_bandwidth = 50 # Increase filter bandwidth for circles

    #10 deg circle - starts at 10 up, fast circle, ends at 10 up (65 waypoints)
    waypoints0 = [-5.0552,-4.5449,-3.9934,-3.4064,-2.7899,-2.1500,-1.4930,-0.8254,-0.1536,0.5159,1.1767,1.8228,2.4479,3.0464,3.6128,4.1417,4.6285,5.0687,5.4583,5.7940,6.0726,6.2917,6.4495,6.5446,6.5761,6.5438,6.4481,6.2899,6.0706,5.7920,5.4568,5.0678,4.6285,4.1428,3.6150,3.0498,2.4523,1.8280,1.1825,0.5220,-0.1475,-0.8196,-1.4878,-2.1455,-2.7863,-3.4038,-3.9917,-4.5441,-5.0552,-5.5197,-5.9328,-6.2900,-6.5875,-6.8223,-6.9917,-7.0940,-7.1281,-7.0936,-6.9909,-6.8212,-6.5863,-6.2887,-5.9317,-5.5191,-5.0552]
    waypoints1 = [-5.0552,-5.5191,-5.9317,-6.2887,-6.5863,-6.8212,-6.9909,-7.0936,-7.1281,-7.0940,-6.9917,-6.8223,-6.5875,-6.2899,-5.9327,-5.5197,-5.0552,-4.5441,-3.9917,-3.4038,-2.7863,-2.1455,-1.4878,-0.8196,-0.1475,0.5220,1.1825,1.8280,2.4523,3.0498,3.6150,4.1428,4.6285,5.0678,5.4568,5.7920,6.0706,6.2899,6.4481,6.5438,6.5761,6.5445,6.4495,6.2917,6.0726,5.7940,5.4583,5.0687,4.6285,4.1417,3.6128,3.0464,2.4479,1.8228,1.1767,0.5158,-0.1537,-0.8254,-1.4930,-2.1500,-2.7899,-3.4064,-3.9934,-4.5449,-5.0552]

    waypoints = [[-waypoints0[-i],-waypoints1[-i]] for i in range(len(waypoints0))]  #-i to flip turn direction

    for i in waypoints:
        odrv0.axis0.controller.input_pos=i[0]
        odrv0.axis1.controller.input_pos=i[1]

        check_state(odrv0)
        if state != 3:
            state_machine(odrv0)
        time.sleep(0.0003)

    #hold at 10 up for a short bit after circle
    odrv0.axis0.controller.input_pos = 5.06  #10 deg up
    odrv0.axis1.controller.input_pos = 5.06
    time.sleep(0.2)

    #reduce bandwidth and return to neutral
    odrv0.axis0.controller.config.input_filter_bandwidth = 2.5 # Decrease filter bandwidth to return to neutral
    odrv0.axis1.controller.config.input_filter_bandwidth = 2.5 # Decrease filter bandwidth to return to neutral
    odrv0.axis0.controller.input_pos = 0
    odrv0.axis1.controller.input_pos = 0

    while True:
        check_state(odrv0)
        if state != 3:
            state_machine(odrv0)
        time.sleep(t_sleep)


##out dated state - ignore
def debug_old(odrv0):
    print("entered debug")
    START_POS_R2 = 0
    START_POS_D2 = 0
    CPR = 8192

    odrv0.axis0.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL
    odrv0.axis0.controller.config.input_mode = INPUT_MODE_TRAP_TRAJ
    odrv0.axis0.trap_traj.config.vel_limit = 10
    odrv0.axis0.trap_traj.config.accel_limit = 10
    odrv0.axis0.trap_traj.config.decel_limit = 10
    odrv0.axis0.motor.config.current_lim = 20

    odrv0.axis1.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL
    odrv0.axis1.controller.config.input_mode = INPUT_MODE_TRAP_TRAJ
    odrv0.axis1.trap_traj.config.vel_limit = 10
    odrv0.axis1.trap_traj.config.accel_limit = 10
    odrv0.axis1.trap_traj.config.decel_limit = 10
    odrv0.axis1.motor.config.current_lim = 20

    commands = [[6,6],[6,-5],[-5,-5],[-5,6]]
    odrv0.axis0.controller.input_pos = 0
    odrv0.axis1.controller.input_pos = 0

    time.sleep(3)

    while True:
        for i in commands:
            print('debug in loop')
            odrv0.axis0.controller.input_pos=i[0]
            odrv0.axis1.controller.input_pos=i[1]
            #add something here to check both motors positions instead of just one
            while True:
            #while !(odrv0.axis0.encoder.shadow_count < (i[0]*CPR)-1000 or odrv0.axis0.encoder.shadow_count > i[0]*(CPR)+1000) and (odrv0.axis1.encoder.shadow_count < (i[1]*CPR)-1000 or odrv0.axis1.encoder.shadow_count > i[1]*(CPR)+1000):
                check_state(odrv0)
                if state != 4:
                    state_machine(odrv0)
                time.sleep(t_sleep)


# modify this as necessary to debug anything
def debug(odrv0):
    print("entered test procedure")
    START_POS_R2 = 0
    START_POS_D2 = 0
    CPR = 8192

    


    odrv0.axis0.motor.config.current_lim = 20
    odrv0.axis0.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL
    odrv0.axis0.controller.config.input_filter_bandwidth = 2 # Set the filter bandwidth [1/s]
    odrv0.axis0.controller.config.input_mode = INPUT_MODE_POS_FILTER # Activate the setpoint filter
    
    odrv0.axis1.motor.config.current_lim = 20
    odrv0.axis1.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL
    odrv0.axis1.controller.config.input_filter_bandwidth = 2 # Set the filter bandwidth [1/s]
    odrv0.axis1.controller.config.input_mode = INPUT_MODE_POS_FILTER # Activate the setpoint filter


    #moving back to centre to prevent big jolts from other states
    odrv0.axis0.controller.input_pos=0
    odrv0.axis1.controller.input_pos=0
    time.sleep(3)

    odrv0.axis0.controller.config.input_filter_bandwidth = 8 # Increase filter bandwidth for circle
    odrv0.axis1.controller.config.input_filter_bandwidth = 8 # Increase filter bandwidth for circle

    waypoints0 = [-6.4577,-5.6811,-4.8557,-3.9901,-3.0931,-2.1735,-1.2403,-0.3026,0.6309,1.5512,2.4500,3.3189,4.1500,4.9358,5.6692,6.3436,6.9531,7.4922,7.9561,8.3410,8.6434,8.8608,8.9915,9.0344,8.9893,8.8568,8.6380,8.3350,7.9505,7.4876,6.9505,6.3436,5.6722,4.9421,4.1595,3.3312,2.4646,1.5674,0.6478,-0.2858,-1.2245,-2.1594,-3.0811,-3.9807,-4.8489,-5.6768,-6.4558,-7.1774,-7.8341,-8.4186,-8.9245,-9.3462,-9.6791,-9.9194,-10.0645,-10.1127,-10.0635,-9.9176,-9.6766,-9.3433,-8.9216,-8.4161,-7.8326]
    waypoints1 = [-7.8326,-8.4161,-8.9216,-9.3433,-9.6766,-9.9176,-10.0635,-10.1127,-10.0645,-9.9194,-9.6791,-9.3462,-8.9245,-8.4185,-7.8341,-7.1774,-6.4557,-5.6768,-4.8489,-3.9807,-3.0811,-2.1593,-1.2245,-0.2858,0.6478,1.5674,2.4646,3.3312,4.1595,4.9421,5.6723,6.3436,6.9505,7.4876,7.9505,8.3350,8.6380,8.8568,8.9893,9.0344,8.9915,8.8608,8.6434,8.3410,7.9561,7.4922,6.9531,6.3436,5.6692,4.9358,4.1500,3.3189,2.4500,1.5512,0.6308,-0.3026,-1.2403,-2.1735,-3.0931,-3.9902,-4.8557,-5.6811,-6.4577]

    waypoints = []
    for i in range(len(waypoints0)):
        waypoints.append([-waypoints0[i], -waypoints1[i]])

    
    

    odrv0.axis0.controller.input_pos = 0
    odrv0.axis1.controller.input_pos = 0

    while True:
        for i in waypoints:
            odrv0.axis0.controller.input_pos=i[0]
            odrv0.axis1.controller.input_pos=i[1]

            check_state(odrv0)
            if state != 4:
                state_machine(odrv0)
            time.sleep(0.1)

        check_state(odrv0)
        if state != 4:
            state_machine(odrv0)
        time.sleep(t_sleep)



def armTVC(odrv0):
    
    print("entered armed")
    #odrv0.axis0.requested_state = AXIS_STATE_IDLE
    #time.sleep(0.1)
    odrv0.axis0.motor.config.current_lim = 30


    odrv0.axis0.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL
    odrv0.axis0.controller.config.input_mode = INPUT_MODE_TRAP_TRAJ
    odrv0.axis0.trap_traj.config.vel_limit = 20
    odrv0.axis0.trap_traj.config.accel_limit = 20
    odrv0.axis0.trap_traj.config.decel_limit = 20

    #odrv0.axis1.requested_state = AXIS_STATE_IDLE
    #time.sleep(0.1)
    odrv0.axis1.motor.config.current_lim = 30

    odrv0.axis1.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL
    odrv0.axis1.controller.config.input_mode = INPUT_MODE_TRAP_TRAJ
    odrv0.axis1.trap_traj.config.vel_limit = 20
    odrv0.axis1.trap_traj.config.accel_limit = 20
    odrv0.axis1.trap_traj.config.decel_limit = 20

    #odrv0.axis0.controller.input_pos = (odrv0.axis0.encoder.shadow_count/8192)
    #odrv0.axis1.controller.input_pos = (odrv0.axis1.encoder.shadow_count/8192)
    odrv0.axis0.controller.input_pos = -9
    odrv0.axis1.controller.input_pos = -9

    while True:
        check_state(odrv0)
        if state != 6:
            state_machine(odrv0)
        time.sleep(t_sleep)

def check_gpio_num(odrv0, num):
    return (odrv0.get_gpio_states() & (1 << num)) != 0


def check_state(odrv0):
    global state,csv_writer
    bit0 = 1 if check_gpio_num(odrv0,6) else 0 #read the gpio pin and set the bit to 0 or 1
    bit1 = 1 if check_gpio_num(odrv0,7) else 0
    bit2 = 1 if check_gpio_num(odrv0,8) else 0
    
    data_row = {"Time_(ns)":time.time_ns(),"State":state,"Shadow_M0": odrv0.axis0.encoder.shadow_count,"Shadow_M1":round(odrv0.axis1.encoder.shadow_count,3),"Current_M0":round(odrv0.axis0.motor.current_control.Iq_measured,3),"Current_M1": round(odrv0.axis1.motor.current_control.Iq_measured,3)}
    csv_writer.writerow(data_row)


    state = (4 * bit2 + 2 * bit1 + bit0)
    #state=0
    print("bit0:", check_gpio_num(odrv0,6), " | bit1:", check_gpio_num(odrv0,7), " | bit2:", check_gpio_num(odrv0,8), " | state:", state, " |  Current", round(odrv0.axis0.motor.current_control.Iq_measured,2), " | Step Count", round(odrv0.axis0.encoder.shadow_count,2), " | Turn Count", round(odrv0.axis0.encoder.shadow_count/8192,2))
    print(" |  Current", round(odrv0.axis1.motor.current_control.Iq_measured,2), " | Step Count", round(odrv0.axis1.encoder.shadow_count,2), " | Turn Count", round(odrv0.axis1.encoder.shadow_count/8192,2))

def state_machine(odrv0):

    check_state(odrv0)
    print("state: ", state)
    if state == 0:
        #armTVC(odrv0)
        idle_state(odrv0)

    if state == 1:
        delay = 0
        print("preparing calibration...")
        while delay < 100:
            idle_state(odrv0)
            delay += 1
        full_reset_and_calibrate(odrv0)

    if state == 2:
        #armTVC(odrv0)
        lock_pos(odrv0,-9,0)

    if state == 3:
        #armTVC(odrv0)
        test_procedure(odrv0)

    if state == 4:
        #armTVC(odrv0)
        debug(odrv0)

    if state == 6:
        armTVC(odrv0)

    if state > 6 or state < 0 or state == 5:
        print("error, incorrect state. Entering idle")
        idle_state(odrv0)

def initialise(odrv0):
    odrv0.config.gpio6_mode = 2 #digital pull down
    odrv0.config.gpio7_mode = 2
    odrv0.config.gpio8_mode = 2

    state_machine(odrv0)





now = datetime.now()
current_time = now.strftime("%H:%M:%S")

with open(f"logs/TEST_LOG_{date.today()}_{current_time}","x") as logfile:
    
    logfile_header = ["Time_(ns)","State","Shadow_M0","Shadow_M1", "Current_M0", "Current_M1"]
    csv_writer = csv.DictWriter(logfile,fieldnames=logfile_header)
    csv_writer.writeheader()

    my_drive = odrive.find_any()
    initialise(my_drive)
