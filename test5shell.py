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
connection_timeout = 1.0  # seconds

calibrated = False

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


async def calibrate(odrv0):
    global state
    print("entered calibration")
    # note: once calibration is entered can't exit until completion
    time.sleep(5)
    calibrated = True
    while True:
        await asyncio.sleep(t_sleep) #calls back to main async loop
        state = message["tvcs"]["tvc0"]["state"]
        last_sent_time = float(time.time()) - float(message["timestamp"])
        print(state, ", last sent time = ", last_sent_time)
        #if last_sent_time > connection_timeout:
            #state = "lock"
            #await lock(odrv0)
        if state != "calibrate":
            await state_machine(odrv0)
            break
            
            


async def test_procedure(odrv0):
    global state
    print("entered test procedure")
    time.sleep(5)

    while True:
        await asyncio.sleep(t_sleep) #calls back to main async loop
        state = message["tvcs"]["tvc0"]["state"]
        last_sent_time = float(time.time()) - float(message["timestamp"])
        print(state, ", last sent time = ", last_sent_time)
        if last_sent_time > connection_timeout:
            state = "lock"
            #await lock(odrv0)
        if state != "test_procedure":
            await state_machine(odrv0)
            break

async def idle_state(odrv0):
    global state
    print("entered idle")
    

    #odrv0.axis0.requested_state = AXIS_STATE_IDLE
    #odrv0.axis1.requested_state = AXIS_STATE_IDLE

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
    #odrv0.axis0.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL
    #odrv0.axis1.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL
    
    while True:
        await asyncio.sleep(t_sleep) #calls back to main async loop
        state = message["tvcs"]["tvc0"]["state"]
        last_sent_time = float(time.time()) - float(message["timestamp"])
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

    while True:
        await asyncio.sleep(t_sleep) #calls back to main async loop
        state = message["tvcs"]["tvc0"]["state"]
        last_sent_time = float(time.time()) - float(message["timestamp"])
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
    
    while True:
        await asyncio.sleep(t_sleep) #calls back to main async loop
        ga0 = message["tvcs"]["tvc0"]["gimbal_angle_0"]
        ga1 = message["tvcs"]["tvc0"]["gimbal_angle_1"]
        gimbal_angles = [ga0, ga1]
        actTurns = TVCKinematics.actuator_lengths_gimbal(gimbal_angles, offset=True, unit_turns=True)
        #odrv0.axis0.controller.input_pos=actTurns[0]
        #odrv0.axis1.controller.input_pos=actTurns[1]

        print("Gimbal angles = ", gimbal_angles)

        state = message["tvcs"]["tvc0"]["state"] # placeholder for state variable in json file
        last_sent_time = float(time.time()) - float(message["timestamp"])
        print(state, ", last sent time = ", last_sent_time)
        if last_sent_time > connection_timeout:
            state = "lock"
            #await lock(odrv0)
        if state != "demand_pos":
            await state_machine(odrv0)
            break


async def watchdog():
    global last_sent_time
    while True:
        await asyncio.sleep(0.001)
        if last_sent_time is not None:
            elapsed = float(time.time()) - last_sent_time
            if elapsed > connection_timeout:
                print(f"Connection timeout: No message received in {elapsed:.1f} seconds")
                await lock(odrv0)
                last_sent_time = None  


async def state_machine(odrv0):
    global state
    #state = message["tvcs"]["tvc0"]["state"] # placeholder for state variable in json file

    #print("state: ", state)
    last_sent_time = float(time.time()) - float(message["timestamp"])
    if last_sent_time > connection_timeout:
        state = "lock"


    if state == "idle":
        await idle_state(odrv0)

    elif state == "calibrate":
        if not calibrated:
            delay = 0
            print("preparing calibration...")
            #while delay < 100:
                #idle_state(odrv0)
                #delay += 1
            await calibrate(odrv0)  
        else:
            await idle_state(odrv0)

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

