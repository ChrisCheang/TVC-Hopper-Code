import odrive
from odrive.enums import *
import time
import datetime
import csv
from datetime import date
from datetime import datetime

from math import *
import numpy as np
from pyquaternion import Quaternion


# stuff for communications with json and udp
import asyncio
import json


rEngine = 32;  # radius of the actuator engine mounts
hTopRing = 23; # axial (z) distance downwards between the pivot point and the engine top ring (bottom edge)
hEngine = 166.9; # axial (z) distance downwards between the pivot point and the engine bottom
lPivot = 106.9; # axial (z) distance downwards between the pivot point and the engine actuator mount points
hMount = 5; # axial (z) distance upwards between the pivot point and the stationary actuator mount points. 17-2: before cut = 63, after should = 63-14.8
rMount = 203; # radius of the stationary actuator mounts, r=120
aMax = 7*np.pi/180; # maximum gimbal angle in radians
lead = 2; # lead of ball screw in mm

global state
state = 0

global t_sleep, csv_writer
t_sleep = 0.001
csv_writer = None

initialized = True
message = None

# For watchdog timer
last_sent_time = None
connection_timeout = 5.0  # seconds

calibrated = False
tested = False

class UDPServerProtocol:
    def connection_made(self, transport):
        self.transport = transport
        print("UDP server ready and listening")

    def datagram_received(self, data, addr):
        global message, last_sent_time
        try:
            message = json.loads(data.decode())
            last_sent_time = float(time.time()) - float(message["timestamp"])
            #print(f"Received JSON from {addr}: {message}") 

            if odrv0 is not None:
                turn_counts = [
                    1.0,#odrv0.axis0.encoder.pos_estimate,
                    2.0,#odrv0.axis1.encoder.pos_estimate
                ]
                currents = [
                    3.0,#odrv0.axis0.motor.current_control.Iq_measured,
                    4.0,#odrv0.axis1.motor.current_control.Iq_measured
                ]

                response = {
                    "turn_counts": turn_counts,
                    "currents": currents,
                    "timestamp": time.time()
                }

                self.transport.sendto(json.dumps(response).encode(), addr)
                #print(f"Sent feedback to {addr}: {response}")
        
        except json.JSONDecodeError:
            print(f"Received invalid JSON from {addr}: {data.decode()}")
    
    def error_received(self, exc):
        print(f"Error received: {exc}")

    def connection_lost(self, exc):
        print("Connection closed")

# at the moment this only includes rotate_gimbal_angles and actuator_lengths_gimbal for the sweep, spherical doesn't seem to be working
class TVCKinematics:

    # rotate any point based on the two gimbal angles, defined positive for movement in the y or x direction respectively
    def rotate_gimbal_angles(gimbal_angles,point):
        outer = Quaternion(axis=np.array([1., 0., 0.]), angle=gimbal_angles[0]) #outer gimbal axis, colinear with x axis

        v_o = outer.rotate(point) # rotated from outer axis angle

        y = np.array([0.,1.,0.]) # original inner gimbal axis
        y_o = outer.rotate(y) # rotated inner gimbal axis

        inner = Quaternion(axis=y_o, angle=-gimbal_angles[1]) #outer gimbal axis
        return inner.rotate(v_o)

    # calculate required actuator lengths to achieve given gimbal angles (inverse kinematics)
    def actuator_lengths_gimbal(gimbal_angles, offset=False, unit_turns=False):
        # Actuator 1 is in the first quadrant, 2 in the second

        # actuator thrust plate mount points
        a1_thrust_plate_mount = np.array([rMount*cos(pi/4),rMount*sin(pi/4),hMount])
        a2_thrust_plate_mount = np.array([rMount*cos(3*pi/4),rMount*sin(3*pi/4),hMount])

        # Non-rotated engine actuator mount points
        a1_engine_mount_original = np.array([rEngine*cos(pi/4),rEngine*sin(pi/4),-lPivot])
        a2_engine_mount_original = np.array([rEngine*cos(3*pi/4),rEngine*sin(3*pi/4),-lPivot])

        # neutral actuator lengths from desired position
        a1length_orig = np.sqrt(np.sum((a1_engine_mount_original-a1_thrust_plate_mount)**2, axis=0))
        a2length_orig = np.sqrt(np.sum((a2_engine_mount_original-a2_thrust_plate_mount)**2, axis=0))

        # rotated actuator mount points
        a1_engine_mount = TVCKinematics.rotate_gimbal_angles(gimbal_angles,a1_engine_mount_original)
        a2_engine_mount = TVCKinematics.rotate_gimbal_angles(gimbal_angles,a2_engine_mount_original)

        # required actuator lengths from desired position
        a1length = np.sqrt(np.sum((a1_engine_mount-a1_thrust_plate_mount)**2, axis=0))
        a2length = np.sqrt(np.sum((a2_engine_mount-a2_thrust_plate_mount)**2, axis=0))

        if offset == True:
            a1length -= a1length_orig
            a2length -= a2length_orig

        if unit_turns == True:
            a1length = a1length/lead
            a2length = a2length/lead

        return [a1length,a2length]


#from fibre.libfibre import ObjectLostError


try: # Reboot causes loss of connection, use try to supress errors
    odrv0.reboot()
except:
    pass

odrv0 = odrive.find_any() # Reconnect to the Odrive


#start full log
#'''
now = datetime.now()
now_ns = time.time_ns()
current_time = now.strftime("%H_%M_%S")
full_file = f"FULL_LOG_{date.today()}_{current_time}.csv"
logfile_header = ["Time_(s)","State","Target_M0","Target_M1","Turns_M0","Turns_M1", "Current_M0", "Current_M1"]

with open(full_file, "a", newline='') as fullfile:
    writer = csv.DictWriter(fullfile, fieldnames=logfile_header)
    writer.writeheader()  

def datafull(odrv0, Target_M0=0, Target_M1=0):
    with open(full_file,"a", newline='') as fullfile: 
        writer = csv.DictWriter(fullfile, fieldnames=logfile_header)
        data_row = {"Time_(s)":round((time.time_ns() - now_ns)/(10**9),3),"State":f"{state}","Target_M0": Target_M0,"Target_M1": Target_M1,"Turns_M0": round(odrv0.axis0.encoder.shadow_count/8192,3),"Turns_M1":round(odrv0.axis1.encoder.shadow_count/8192,3),"Current_M0":round(odrv0.axis0.motor.current_control.Iq_measured,3),"Current_M1": round(odrv0.axis1.motor.current_control.Iq_measured,3)}
        writer.writerow(data_row)
#'''


async def calibrate(odrv0):
    global state, calibrated
    if not calibrated:
        print("entered calibration")

        #'''
        #odrv0.reboot()
        odrv0.clear_errors()
        

        odrv0.axis0.min_endstop.config.enabled = False
        odrv0.axis1.min_endstop.config.enabled = False

        odrv0.axis0.encoder.config.cpr = 8192
        odrv0.axis1.encoder.config.cpr = 8192

        odrv0.axis0.motor.config.pole_pairs = 11
        odrv0.axis1.motor.config.pole_pairs = 14 #remember to make sure this is correct!

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
        odrv0.axis0.motor.config.current_lim = 5
        odrv0.axis0.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL
        odrv0.axis0.trap_traj.config.vel_limit = 1
        odrv0.axis0.trap_traj.config.accel_limit = 1
        odrv0.axis0.trap_traj.config.decel_limit = 1
        odrv0.axis0.controller.config.input_mode = INPUT_MODE_TRAP_TRAJ
        odrv0.axis0.controller.input_pos = 100
        
        while round(odrv0.axis0.motor.current_control.Iq_measured,2)<4: # Wait for calibration to be done
            print("current: ",round(odrv0.axis0.motor.current_control.Iq_measured,2))
            
        odrv0.axis0.encoder.set_linear_count(int(14.5*8192))
        odrv0.axis0.requested_state = AXIS_STATE_IDLE
        

        print("Motor 0: Homing complete")

        time.sleep(0.5) #time sleep could help prevent sudden changes


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
        odrv0.axis1.motor.config.current_lim = 5
        odrv0.axis1.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL
        odrv0.axis1.trap_traj.config.vel_limit = 1
        odrv0.axis1.trap_traj.config.accel_limit = 1
        odrv0.axis1.trap_traj.config.decel_limit = 1
        odrv0.axis1.controller.config.input_mode = INPUT_MODE_TRAP_TRAJ
        odrv0.axis1.controller.input_pos = 100

        while round(odrv0.axis1.motor.current_control.Iq_measured,2)<4: # Wait for calibration to be done
            print("current: ",round(odrv0.axis1.motor.current_control.Iq_measured,2))
        
        odrv0.axis1.encoder.set_linear_count(int(14.5*8192))
        odrv0.axis1.requested_state = AXIS_STATE_IDLE

        time.sleep(0.5)#time sleep could help prevent sudden changes


        print("Motor 1: Homing complete")

        print("Movingt to middle")

        odrv0.axis0.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL
        odrv0.axis1.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL

        odrv0.axis0.controller.config.input_mode = INPUT_MODE_TRAP_TRAJ
        odrv0.axis0.trap_traj.config.vel_limit = 5
        odrv0.axis0.trap_traj.config.accel_limit = 5
        odrv0.axis0.trap_traj.config.decel_limit = 5

        odrv0.axis1.controller.config.input_mode = INPUT_MODE_TRAP_TRAJ
        odrv0.axis1.trap_traj.config.vel_limit = 5
        odrv0.axis1.trap_traj.config.accel_limit = 5
        odrv0.axis1.trap_traj.config.decel_limit = 5


        odrv0.axis0.controller.input_pos = 0
        odrv0.axis1.controller.input_pos = 0

        time.sleep(5)

        odrv0.axis0.requested_state = AXIS_STATE_IDLE
        odrv0.axis1.requested_state = AXIS_STATE_IDLE

        
        #'''
        time.sleep(5)
        print("Calibrated")
        calibrated = True



    else:
        print("calibrated, idling")
        odrv0.axis0.requested_state = AXIS_STATE_IDLE
        odrv0.axis1.requested_state = AXIS_STATE_IDLE
    while True:
        await asyncio.sleep(t_sleep) #calls back to main async loop
        state = message["tvcs"]["tvc0"]["state"]
        last_sent_time = float(time.time()) - float(message["timestamp"])
        print("Calibrated: ", calibrated, ", last sent time = ", last_sent_time)
        #if last_sent_time > connection_timeout:
            #state = "lock"
            #await lock(odrv0)
        if state != "calibrate":
            await state_machine(odrv0)
            break
            
            


async def test_procedure(odrv0):
    global state, tested
    print("entered test procedure")


    if not tested:
        print("open csv data log")
        #'''

        now = datetime.now()
        now_ns = time.time_ns()
        current_time = now.strftime("%H_%M_%S")
        test_file = f"TEST_LOG_{date.today()}_{current_time}.csv"
        logfile_header = ["Time_(s)","Target_M0","Target_M1","Turns_M0","Turns_M1", "Current_M0", "Current_M1"]

        with open(test_file, "a", newline='') as logfile:
            writer = csv.DictWriter(logfile, fieldnames=logfile_header)
            writer.writeheader()  

        def data(odrv0, Target_M0=0, Target_M1=0):
            with open(test_file,"a", newline='') as logfile: 
                writer = csv.DictWriter(logfile, fieldnames=logfile_header)
                data_row = {"Time_(s)":round((time.time_ns() - now_ns)/(10**9),3),"Target_M0": Target_M0,"Target_M1": Target_M1,"Turns_M0": round(odrv0.axis0.encoder.shadow_count/8192,3),"Turns_M1":round(odrv0.axis1.encoder.shadow_count/8192,3),"Current_M0":round(odrv0.axis0.motor.current_control.Iq_measured,3),"Current_M1": round(odrv0.axis1.motor.current_control.Iq_measured,3)}
                writer.writerow(data_row)
            print("M0 Current", round(odrv0.axis0.motor.current_control.Iq_measured,2), " | Turn Count", round(odrv0.axis0.encoder.shadow_count/8192,2), "M1 Current", round(odrv0.axis1.motor.current_control.Iq_measured,2), " | Turn Count", round(odrv0.axis1.encoder.shadow_count/8192,2))

        # set parameters

        odrv0.axis0.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL
        odrv0.axis1.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL

        odrv0.axis0.motor.config.current_lim = 30
        odrv0.axis1.motor.config.current_lim = 30

        odrv0.axis0.controller.config.input_mode = INPUT_MODE_POS_FILTER # Activate the setpoint filter
        odrv0.axis1.controller.config.input_mode = INPUT_MODE_POS_FILTER # Activate the setpoint filter

        # check with the ones in the current config first - before today both 2, 15 is highest tested
        odrv0.axis0.controller.config.input_filter_bandwidth = 16
        odrv0.axis1.controller.config.input_filter_bandwidth = 16

        #odrv0.axis0.controller.config.input_mode = INPUT_MODE_TRAP_TRAJ
        #odrv0.axis0.trap_traj.config.vel_limit = 100000
        #odrv0.axis0.trap_traj.config.accel_limit = 900
        #odrv0.axis0.trap_traj.config.decel_limit = 900

        #odrv0.axis1.controller.config.input_mode = INPUT_MODE_TRAP_TRAJ
        #odrv0.axis1.trap_traj.config.vel_limit = 100000
        #odrv0.axis1.trap_traj.config.accel_limit = 900
        #odrv0.axis1.trap_traj.config.decel_limit = 900


        
        # 1. Up, circle, down
        # waypoints from ThrustVectMockup1 matlab - need to fix kinematics.py
        amax = 7*3.14159/180; # maximum gimbal angle in radians

        turns0 = [0 for i in range(50)]+[0, -0.46, -0.93, -1.39, -1.86, -2.32, -2.79, -3.26, -3.73, -4.2, -4.67, -4.18, -3.66, -3.1, -2.52, -1.91, -1.28, -0.65, -0.01, 0.63, 1.26, 1.88, 2.47, 3.04, 3.59, 4.09, 4.56, 4.98, 5.35, 5.67, 5.94, 6.15, 6.3, 6.39, 6.43, 6.39, 6.3, 6.15, 5.94, 5.67, 5.35, 4.98, 4.56, 4.09, 3.59, 3.04, 2.47, 1.88, 1.26, 0.63, -0.01, -0.65, -1.28, -1.91, -2.52, -3.1, -3.66, -4.18, -4.67, -5.11, -5.5, -5.83, -6.11, -6.34, -6.5, -6.59, -6.62, -6.59, -6.5, -6.34, -6.11, -5.83, -5.5, -5.11, -4.67, -4.2, -3.73, -3.26, -2.79, -2.32, -1.86, -1.39, -0.93, -0.46, 0]
        turns1 = [0 for i in range(50)]+[0, -0.46, -0.93, -1.39, -1.86, -2.32, -2.79, -3.26, -3.73, -4.2, -4.67, -5.11, -5.5, -5.83, -6.11, -6.34, -6.5, -6.59, -6.62, -6.59, -6.5, -6.34, -6.11, -5.83, -5.5, -5.11, -4.67, -4.18, -3.66, -3.1, -2.52, -1.91, -1.28, -0.65, -0.01, 0.63, 1.26, 1.88, 2.47, 3.04, 3.59, 4.09, 4.56, 4.98, 5.35, 5.67, 5.94, 6.15, 6.3, 6.39, 6.43, 6.39, 6.3, 6.15, 5.94, 5.67, 5.35, 4.98, 4.56, 4.09, 3.59, 3.04, 2.47, 1.88, 1.26, 0.63, -0.01, -0.65, -1.28, -1.91, -2.52, -3.1, -3.66, -4.18, -4.67, -4.2, -3.73, -3.26, -2.79, -2.32, -1.86, -1.39, -0.93, -0.46, 0]
        
        #turns0 = [-4.67, -4.18, -3.66, -3.1, -2.52, -1.91, -1.28, -0.65, -0.01, 0.63, 1.26, 1.88, 2.47, 3.04, 3.59, 4.09, 4.56, 4.98, 5.35, 5.67, 5.94, 6.15, 6.3, 6.39, 6.43, 6.39, 6.3, 6.15, 5.94, 5.67, 5.35, 4.98, 4.56, 4.09, 3.59, 3.04, 2.47, 1.88, 1.26, 0.63, -0.01, -0.65, -1.28, -1.91, -2.52, -3.1, -3.66, -4.18, -4.67, -5.11, -5.5, -5.83, -6.11, -6.34, -6.5, -6.59, -6.62, -6.59, -6.5, -6.34, -6.11, -5.83, -5.5, -5.11, -4.67]
        #urns1 = [-4.67, -5.11, -5.5, -5.83, -6.11, -6.34, -6.5, -6.59, -6.62, -6.59, -6.5, -6.34, -6.11, -5.83, -5.5, -5.11, -4.67, -4.18, -3.66, -3.1, -2.52, -1.91, -1.28, -0.65, -0.01, 0.63, 1.26, 1.88, 2.47, 3.04, 3.59, 4.09, 4.56, 4.98, 5.35, 5.67, 5.94, 6.15, 6.3, 6.39, 6.43, 6.39, 6.3, 6.15, 5.94, 5.67, 5.35, 4.98, 4.56, 4.09, 3.59, 3.04, 2.47, 1.88, 1.26, 0.63, -0.01, -0.65, -1.28, -1.91, -2.52, -3.1, -3.66, -4.18, -4.67]
        

        currents = []
        turns = []
        for i in range(len(turns0)):
            turns.append([turns0[i], turns1[i]])


        for i in turns:
            data(odrv0, i[0], i[1])
            datafull(odrv0, i[0], i[1])
            #print("current0: ",round(odrv0.axis0.motor.current_control.Iq_measured,2))
            #print("current1: ",round(odrv0.axis1.motor.current_control.Iq_measured,2))
            currents.append(max(odrv0.axis0.motor.current_control.Iq_measured,odrv0.axis1.motor.current_control.Iq_measured))
            odrv0.axis0.controller.input_pos=i[0]
            odrv0.axis1.controller.input_pos=i[1]
            await asyncio.sleep(0.005) #calls back to main async loop
            state = message["tvcs"]["tvc0"]["state"]
            last_sent_time = float(time.time()) - float(message["timestamp"])
            #print(state, ", last sent time = ", last_sent_time)
            if last_sent_time > connection_timeout:
                state = "lock"
                #await lock(odrv0)
            if state != "test_procedure":
                await state_machine(odrv0)
                break
            


        print("Max current hit: ", max(currents))

        odrv0.axis0.controller.input_pos = 0
        odrv0.axis1.controller.input_pos = 0
        #'''


        #'''
        # 2. x axis step sweep (inner gimbal)
        # note on the kinematics: would probably be less load on the pi by feeding waypoints, but the kinematics have to be done in real time anyways for flight
        def sinesweep(x,a=0.5,amax=aMax):
            # function for defining sinesweep - which will then be converted to "step sweep"
            # x is time in s, a affects frequency, amax is max angle

            x = np.array(x)  # allows vectorized input
            result = np.zeros_like(x, dtype=float)

            # Apply the sinusoidal function for 2 < x < 12
            mask = (x > 2) & (x < 12)
            result[mask] = amax * np.sin(np.pi * (a * (x[mask] - 2) + 1)**2)

            return result
        
        def stepsweep(x, a=0.5, amax=aMax): return amax*np.sign(sinesweep(x, a=a, amax=aMax))

        start = datetime.now()
        t = 0

        currents = []

        zero_target = []
        one_target = []

        zero_pos = []
        one_pos = []

        times = []

        while t < 12:
            t = datetime.now() - start
            t = t.total_seconds()

            # change below to change sweep type, [0,stepsweep(t)] is x sweep, [stepsweep(t),0] y sweep, [0.707*stepsweep(t),0.707*stepsweep(t)] act 0 sweep, [0.707*stepsweep(t),-0.707*stepsweep(t)] act 1 sweep
            gimbal_angles = [0.707*stepsweep(t),0.707*stepsweep(t)]    
            actTurns = TVCKinematics.actuator_lengths_gimbal(gimbal_angles, offset=True, unit_turns=True)
            
            odrv0.axis0.controller.input_pos=actTurns[0]
            odrv0.axis1.controller.input_pos=actTurns[1]

            currents.append(max(odrv0.axis0.motor.current_control.Iq_measured,odrv0.axis1.motor.current_control.Iq_measured))

            #print("Actuator target turns: ", actTurns[0],", ", actTurns[1])
            #print("current0: ",round(odrv0.axis0.motor.current_control.Iq_measured,2))
            #print("current1: ",round(odrv0.axis1.motor.current_control.Iq_measured,2))
            data(odrv0, actTurns[0], actTurns[1])
            datafull(odrv0, actTurns[0], actTurns[1])

            #data collection
            times.append(round(t,3))
            zero_target.append(round(float(actTurns[0]),2))
            one_target.append(round(float(actTurns[1]),2))
            zero_pos.append(round(float(odrv0.axis0.encoder.shadow_count/8192),2))
            one_pos.append(round(float(odrv0.axis1.encoder.shadow_count/8192),2))

            await asyncio.sleep(t_sleep) #calls back to main async loop
            state = message["tvcs"]["tvc0"]["state"]
            last_sent_time = float(time.time()) - float(message["timestamp"])
            #print(state, ", last sent time = ", last_sent_time)
            if last_sent_time > connection_timeout:
                state = "lock"
                #await lock(odrv0)
            if state != "test_procedure":
                await state_machine(odrv0)
                break
            

        
        x_sweep_max_current = max(currents)    
        print("Max current hit: ", x_sweep_max_current)

        odrv0.axis0.controller.input_pos = 0
        odrv0.axis1.controller.input_pos = 0
        
        #'''

        print("test procedure complete")
        tested = True
    

    while True:
        await asyncio.sleep(t_sleep) #calls back to main async loop
        state = message["tvcs"]["tvc0"]["state"]
        last_sent_time = float(time.time()) - float(message["timestamp"])
        print("Tested: ", tested, ", last sent time = ", last_sent_time)
        datafull(odrv0)
        if last_sent_time > connection_timeout:
            state = "lock"
            #await lock(odrv0)
        if state != "test_procedure":
            await state_machine(odrv0)
            break



async def idle_state(odrv0):
    global state
    print("entered idle")
    
    odrv0.axis0.requested_state = AXIS_STATE_IDLE
    odrv0.axis1.requested_state = AXIS_STATE_IDLE

    while True:
        await asyncio.sleep(t_sleep) #calls back to main async loop
        state = message["tvcs"]["tvc0"]["state"]
        last_sent_time = float(time.time()) - float(message["timestamp"])
        print(state, ", last sent time = ", last_sent_time)
        if last_sent_time > connection_timeout:
            state = "lock"
            #await lock(odrv0)
        if state != "idle":
            print("in idle, errornous state: ", state)
        if state in ("calibrate", "arm", "lock", "test_procedure", "demand_pos"):
            await state_machine(odrv0)
            break


async def lock(odrv0):
    global state
    print("locking position")
    odrv0.axis0.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL
    odrv0.axis1.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL
    
    while True:
        await asyncio.sleep(t_sleep) #calls back to main async loop
        state = message["tvcs"]["tvc0"]["state"]
        last_sent_time = float(time.time()) - float(message["timestamp"])
        datafull(odrv0)
        if last_sent_time > connection_timeout:
            print("Locked, connection timeout, last sent time = ", round(last_sent_time,3), ", last input state: ", state)
        else:
            print(state, ", last sent time = ", last_sent_time)
        if state != "lock" and last_sent_time < connection_timeout:
            await state_machine(odrv0)
            break


async def armTVC(odrv0):
    global state
    print("entered armed")
    #'''

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
    odrv0.axis0.controller.input_pos = 0
    odrv0.axis1.controller.input_pos = 0

    #'''

    while True:
        await asyncio.sleep(t_sleep) #calls back to main async loop
        state = message["tvcs"]["tvc0"]["state"]
        last_sent_time = float(time.time()) - float(message["timestamp"])
        datafull(odrv0)
        print(state, ", last sent time = ", last_sent_time)
        if last_sent_time > connection_timeout:
            state = "lock"
            #await lock(odrv0)
        if state != "arm":
            await state_machine(odrv0)
            break


async def demand_pos(odrv0):
    global state
    print("entered continous demand position mode")

    #'''
    
    odrv0.axis0.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL
    odrv0.axis1.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL

    odrv0.axis0.motor.config.current_lim = 30
    odrv0.axis1.motor.config.current_lim = 30

    odrv0.axis0.controller.config.input_mode = INPUT_MODE_POS_FILTER # Activate the setpoint filter
    odrv0.axis1.controller.config.input_mode = INPUT_MODE_POS_FILTER # Activate the setpoint filter

    # check with the ones in the current config first - before today both 2, 15 is highest tested
    odrv0.axis0.controller.config.input_filter_bandwidth = 16
    odrv0.axis1.controller.config.input_filter_bandwidth = 16

    #'''

    while True:
        await asyncio.sleep(t_sleep) #calls back to main async loop
        ga0 = message["tvcs"]["tvc0"]["gimbal_angle_0"]
        ga1 = message["tvcs"]["tvc0"]["gimbal_angle_1"]
        gimbal_angles = [ga0, ga1]
        actTurns = TVCKinematics.actuator_lengths_gimbal(gimbal_angles, offset=True, unit_turns=True)
        odrv0.axis0.controller.input_pos=actTurns[0]
        odrv0.axis1.controller.input_pos=actTurns[1]

        print("Gimbal angles = ", gimbal_angles)

        state = message["tvcs"]["tvc0"]["state"] # placeholder for state variable in json file
        last_sent_time = float(time.time()) - float(message["timestamp"])
        datafull(odrv0, actTurns[0], actTurns[1])
        print(state, ", last sent time = ", last_sent_time)
        if last_sent_time > connection_timeout:
            state = "lock"
            await lock(odrv0)
        if state != "demand_pos":
            await state_machine(odrv0)
            break


async def state_machine(odrv0):
    global state
    state = message["tvcs"]["tvc0"]["state"] # placeholder for state variable in json file

    print("state: ", state)
    last_sent_time = float(time.time()) - float(message["timestamp"])
    if last_sent_time > connection_timeout:
        state = "lock"


    if state == "idle":
        await idle_state(odrv0)

    elif state == "calibrate":
        delay = 0
        print("preparing calibration...")
        #while delay < 100:
            #idle_state(odrv0)
            #delay += 1
        await calibrate(odrv0)  

    elif state == "arm":
        await armTVC(odrv0)

    elif state == "lock":
        await lock(odrv0)

    elif state == "test_procedure":
        await test_procedure(odrv0)

    elif state == "demand_pos":
        await demand_pos(odrv0)

    else:
        print("error, incorrect state. Entering idle")
        await idle_state(odrv0)


async def main():
    print("Starting UDP server...")
    loop = asyncio.get_running_loop()
    transport, protocol = await loop.create_datagram_endpoint(
        lambda: UDPServerProtocol(),
        local_addr=('127.0.0.1', 9999)
    )

    #asyncio.create_task(watchdog())

    while message is None:
        print("Waiting for initial JSON input...")
        await asyncio.sleep(0.1)

    await state_machine(odrv0)
    

    try:
        await asyncio.sleep(3600)  # Keep server running for 1 hour
    finally:
        transport.close()

if __name__ == "__main__":
    asyncio.run(main())
