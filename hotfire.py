import odrive
from odrive.enums import *
import numpy as np
import time
#from fibre.libfibre import ObjectLostError

global state
state = 0

global t_sleep
t_sleep = 0.05

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
    odrv0.axis0.min_endstop.config.offset = -13.7  #-10.5
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
    odrv0.axis1.min_endstop.config.offset = -13.7  #-10.5
    odrv0.axis1.min_endstop.config.enabled = True

    #homing setup
    odrv0.axis1.controller.config.homing_speed = -2
    odrv0.axis1.controller.config.vel_ramp_rate = 0.5
    odrv0.axis1.trap_traj.config.vel_limit = 2
    odrv0.axis1.trap_traj.config.accel_limit = 2
    odrv0.axis1.trap_traj.config.decel_limit = 2

    initializeRCPWM(odrv0)


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

    


def lock_pos(odrv0,r2_pos,d2_pos):
    print("entered lock pos")
    odrv0.axis0.requested_state = AXIS_STATE_IDLE
    time.sleep(0.1)
    odrv0.axis0.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL
    odrv0.axis0.controller.config.input_mode = INPUT_MODE_TRAP_TRAJ
    odrv0.axis0.trap_traj.config.vel_limit = 20
    odrv0.axis0.trap_traj.config.accel_limit = 20
    odrv0.axis0.trap_traj.config.decel_limit = 20

    odrv0.axis1.requested_state = AXIS_STATE_IDLE
    time.sleep(0.1)
    odrv0.axis1.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL
    odrv0.axis1.controller.config.input_mode = INPUT_MODE_TRAP_TRAJ
    odrv0.axis1.trap_traj.config.vel_limit = 20
    odrv0.axis1.trap_traj.config.accel_limit = 20
    odrv0.axis1.trap_traj.config.decel_limit = 20

    #odrv0.axis0.controller.input_pos = (odrv0.axis0.encoder.shadow_count/8192)
    #odrv0.axis1.controller.input_pos = (odrv0.axis0.encoder.shadow_count/8192)
    odrv0.axis0.controller.input_pos = 0
    odrv0.axis1.controller.input_pos = 0

    while True:
        check_state(odrv0)
        if state != 2:
            state_machine(odrv0)
        time.sleep(t_sleep)



def test_procedure_alt(odrv0):
    print("entered test procedure")
    START_POS_R2 = 0
    START_POS_D2 = 0
    CPR = 8192

    odrv0.axis0.motor.config.current_lim = 20
    odrv0.axis0.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL
    odrv0.axis0.controller.config.input_filter_bandwidth = 4 # Set the filter bandwidth [1/s]
    odrv0.axis0.controller.config.input_mode = INPUT_MODE_POS_FILTER # Activate the setpoint filter
    
    odrv0.axis1.motor.config.current_lim = 20
    odrv0.axis1.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL
    odrv0.axis1.controller.config.input_filter_bandwidth = 4 # Set the filter bandwidth [1/s]
    odrv0.axis1.controller.config.input_mode = INPUT_MODE_POS_FILTER # Activate the setpoint filter

    #moving back to centre to prevent big jolts from other states
    odrv0.axis0.controller.input_pos=0
    odrv0.axis1.controller.input_pos=0
    time.sleep(3)

    commands = [[-10,0],[-7.5,7.5],[0,10],[7.5,7.5],[10,0],[7.5,-7.5],[0,-10],[-7.5,-7.5],[-10,0],[-7.5,7.5],[0,10],[7.5,7.5],[10,0],[7.5,-7.5],[0,-10],[-7.5,-7.5]]  

    for i in commands:
        odrv0.axis0.controller.input_pos=i[0]
        odrv0.axis1.controller.input_pos=i[1]
        #add something here to check both motors positions instead of just one
        check_state(odrv0)
        if state != 4:
            state_machine(odrv0)
        time.sleep(0.2) #0.2 for fast

    odrv0.axis0.controller.input_pos=0
    odrv0.axis1.controller.input_pos=0

    
def test_procedure(odrv0):
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

    waypoints0 = [0,-0.1,-0.2,-0.4784,-0.9616,-1.4496,-1.9422,-2.4395,-2.9414,-3.4477,-3.9584,-4.4734,-4.9925,-4.4935,-3.9541,-3.3798,-2.7764,-2.1498,-1.5063,-0.8521,-0.1935,0.4631,1.1115,1.7456,2.3594,2.9472,3.5036,4.0234,4.5018,4.9346,5.3177,5.6477,5.9216,6.1371,6.2922,6.3856,6.4166,6.3848,6.2906,6.1350,5.9192,5.6454,5.3158,4.9335,4.5018,4.0247,3.5063,2.9513,2.3648,1.7520,1.1187,0.4707,-0.1858,-0.8447,-1.4996,-2.1440,-2.7717,-3.3764,-3.9520,-4.4925,-4.9925,-5.4468,-5.8507,-6.1998,-6.4905,-6.7198,-6.8853,-6.9851,-7.0183,-6.9845,-6.8842,-6.7183,-6.4888,-6.1981,-5.8493,-5.4460,-4.9925,-4.4733,-3.9584,-3.4477,-2.9414,-2.4395,-1.9422,-1.4495,-0.9616,-0.4784,0]
    waypoints1 = [0,-0.1,-0.2,-0.4784,-0.9616,-1.4496,-1.9423,-2.4396,-2.9414,-3.4477,-3.9584,-4.4734,-4.9925,-5.4460,-5.8493,-6.1981,-6.4888,-6.7183,-6.8842,-6.9845,-7.0183,-6.9851,-6.8853,-6.7198,-6.4905,-6.1998,-5.8506,-5.4468,-4.9925,-4.4925,-3.9519,-3.3764,-2.7717,-2.1440,-1.4996,-0.8447,-0.1858,0.4708,1.1187,1.7520,2.3648,2.9513,3.5063,4.0247,4.5018,4.9335,5.3158,5.6454,5.9192,6.1350,6.2906,6.3848,6.4166,6.3856,6.2922,6.1371,5.9216,5.6477,5.3177,4.9346,4.5018,4.0234,3.5036,2.9472,2.3594,1.7456,1.1115,0.4631,-0.1935,-0.8521,-1.5063,-2.1498,-2.7764,-3.3799,-3.9542,-4.4935,-4.9925,-4.4733,-3.9584,-3.4477,-2.9414,-2.4395,-1.9422,-1.4495,-0.9616,-0.4784,0]

    waypoints = []
    for i in range(len(waypoints0)):
        waypoints.append([waypoints0[i], waypoints1[i]])


    for i in waypoints:
        odrv0.axis0.controller.input_pos=i[0]
        odrv0.axis1.controller.input_pos=i[1]

        check_state(odrv0)
        if state != 3:
            state_machine(odrv0)
        time.sleep(0.05)

    odrv0.axis0.controller.input_pos = 0
    odrv0.axis1.controller.input_pos = 0

    while True:
        check_state(odrv0)
        if state != 3:
            state_machine(odrv0)
        time.sleep(t_sleep)


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

def debug(odrv0):
    odrv0.axis0.motor.config.current_lim = 20
    odrv0.axis0.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL
    odrv0.axis0.controller.config.input_filter_bandwidth = 4 # Set the filter bandwidth [1/s]
    odrv0.axis0.controller.config.input_mode = INPUT_MODE_POS_FILTER # Activate the setpoint filter
    
    odrv0.axis1.motor.config.current_lim = 20
    odrv0.axis1.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL
    odrv0.axis1.controller.config.input_filter_bandwidth = 4 # Set the filter bandwidth [1/s]
    odrv0.axis1.controller.config.input_mode = INPUT_MODE_POS_FILTER # Activate the setpoint filter

    #commands = [[-7,0],[-5.5,5.5],[0,7],[5.5,5.5],[7,0],[5.5,-5.5],[0,-7],[-5.5,-5.5]]  
    commands = [[-10,0],[-7.5,7.5],[0,10],[7.5,7.5],[10,0],[7.5,-7.5],[0,-10],[-7.5,-7.5]]  

    #waypoints0 = [0,-0.4784,-0.9616,-1.4496,-1.9422,-2.4395,-2.9414,-3.4477,-3.9584,-4.4734,-4.9925,-4.4935,-3.9541,-3.3798,-2.7764,-2.1498,-1.5063,-0.8521,-0.1935,0.4631,1.1115,1.7456,2.3594,2.9472,3.5036,4.0234,4.5018,4.9346,5.3177,5.6477,5.9216,6.1371,6.2922,6.3856,6.4166,6.3848,6.2906,6.1350,5.9192,5.6454,5.3158,4.9335,4.5018,4.0247,3.5063,2.9513,2.3648,1.7520,1.1187,0.4707,-0.1858,-0.8447,-1.4996,-2.1440,-2.7717,-3.3764,-3.9520,-4.4925,-4.9925,-5.4468,-5.8507,-6.1998,-6.4905,-6.7198,-6.8853,-6.9851,-7.0183,-6.9845,-6.8842,-6.7183,-6.4888,-6.1981,-5.8493,-5.4460,-4.9925,-4.4733,-3.9584,-3.4477,-2.9414,-2.4395,-1.9422,-1.4495,-0.9616,-0.4784,0]
    #waypoints1 = [0,-0.4784,-0.9616,-1.4496,-1.9423,-2.4396,-2.9414,-3.4477,-3.9584,-4.4734,-4.9925,-5.4460,-5.8493,-6.1981,-6.4888,-6.7183,-6.8842,-6.9845,-7.0183,-6.9851,-6.8853,-6.7198,-6.4905,-6.1998,-5.8506,-5.4468,-4.9925,-4.4925,-3.9519,-3.3764,-2.7717,-2.1440,-1.4996,-0.8447,-0.1858,0.4708,1.1187,1.7520,2.3648,2.9513,3.5063,4.0247,4.5018,4.9335,5.3158,5.6454,5.9192,6.1350,6.2906,6.3848,6.4166,6.3856,6.2922,6.1371,5.9216,5.6477,5.3177,4.9346,4.5018,4.0234,3.5036,2.9472,2.3594,1.7456,1.1115,0.4631,-0.1935,-0.8521,-1.5063,-2.1498,-2.7764,-3.3799,-3.9542,-4.4935,-4.9925,-4.4733,-3.9584,-3.4477,-2.9414,-2.4395,-1.9422,-1.4495,-0.9616,-0.4784,0]

    # while True:
    #     check_state(odrv0)
    #     if state != 4:
    #         state_machine(odrv0)
    #     time.sleep(0.2)

    while True:
        for i in commands:
            odrv0.axis0.controller.input_pos=i[0]
            odrv0.axis1.controller.input_pos=i[1]
            #add something here to check both motors positions instead of just one
            check_state(odrv0)
            if state != 4:
                state_machine(odrv0)
            time.sleep(0.2) #0.2 for fast



def check_gpio_num(odrv0, num):
    return (odrv0.get_gpio_states() & (1 << num)) != 0

def check_state(odrv0):
    global state
    bit0 = 1 if check_gpio_num(odrv0,6) else 0 #read the gpio pin and set the bit to 0 or 1
    bit1 = 1 if check_gpio_num(odrv0,7) else 0
    bit2 = 1 if check_gpio_num(odrv0,8) else 0

    state = (4 * bit2 + 2 * bit1 + bit0)
    print("bit0:", check_gpio_num(odrv0,6), " | bit1:", check_gpio_num(odrv0,7), " | bit2:", check_gpio_num(odrv0,8), " | state:", state, " |  Current", round(odrv0.axis0.motor.current_control.Iq_measured,2), " | Step Count", round(odrv0.axis0.encoder.shadow_count,2), " | Turn Count", round(odrv0.axis0.encoder.shadow_count/8192,2))
    print(" |  Current", round(odrv0.axis1.motor.current_control.Iq_measured,2), " | Step Count", round(odrv0.axis1.encoder.shadow_count,2), " | Turn Count", round(odrv0.axis1.encoder.shadow_count/8192,2))

def state_machine(odrv0):

    check_state(odrv0)
    print("state: ", state)

    if state == 0:
        idle_state(odrv0)

    if state == 1:
        delay = 0
        print("preparing calibration...")
        while delay < 100:
            idle_state(odrv0)
            delay += 1
        full_reset_and_calibrate(odrv0)

    if state == 2:
        lock_pos(odrv0,-9,0)

    if state == 3:
        test_procedure(odrv0)

    if state == 4:
        debug(odrv0)
        # pwmState(odrv0)

    if state > 4 or state < 0:
        print("error, incorrect state. Entering idle")
        idle_state(odrv0)

def initialise(odrv0):
    odrv0.config.gpio6_mode = GPIO_MODE_DIGITAL
    odrv0.config.gpio7_mode = GPIO_MODE_DIGITAL
    odrv0.config.gpio8_mode = GPIO_MODE_DIGITAL

    odrv0.config.gpio6_mode = GPIO_MODE_DIGITAL_PULL_DOWN
    odrv0.config.gpio7_mode = GPIO_MODE_DIGITAL_PULL_DOWN
    odrv0.config.gpio8_mode = GPIO_MODE_DIGITAL_PULL_DOWN

    state_machine(odrv0)

def initializeRCPWM(odrv0):
    odrv0.config.gpio3_mode = GPIO_MODE_PWM
    odrv0.config.gpio3_pwm_mapping.min = -10
    odrv0.config.gpio3_pwm_mapping.max = 10
    odrv0.config.gpio3_pwm_mapping.endpoint = odrv0.axis1.controller._input_pos_property

    odrv0.config.gpio2_mode = GPIO_MODE_PWM
    odrv0.config.gpio2_pwm_mapping.min = -10
    odrv0.config.gpio2_pwm_mapping.max = 10
    odrv0.config.gpio2_pwm_mapping.endpoint = odrv0.axis0.controller._input_pos_property

def pwmState(odrv0):
    # initializeRCPWM(odrv0)

    # try: # Reboot causes loss of connection, use try to supress errors
    #     odrv0.save_configuration()
    #     odrv0.reboot()
    # except:
    #     pass
    # odrv0 = odrive.find_any() # Reconnect to the Odrive

    # print("Odrive: PWM config saved")
    # time.sleep(2)
    # print('restarted')

    odrv0.axis0.motor.config.current_lim = 20
    odrv0.axis0.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL
    odrv0.axis0.controller.config.input_filter_bandwidth = 5 # Set the filter bandwidth [1/s]
    odrv0.axis0.controller.config.input_mode = INPUT_MODE_POS_FILTER # Activate the setpoint filter
    
    odrv0.axis1.motor.config.current_lim = 20
    odrv0.axis1.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL
    odrv0.axis1.controller.config.input_filter_bandwidth = 5 # Set the filter bandwidth [1/s]
    odrv0.axis1.controller.config.input_mode = INPUT_MODE_POS_FILTER # Activate the setpoint filter

    while True:
        check_state(odrv0)
        if state != 4:
            state_machine(odrv0)
        time.sleep(0.2)





my_drive = odrive.find_any()

initialise(my_drive)
