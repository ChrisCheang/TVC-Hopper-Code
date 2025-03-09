from math import *
import numpy as np
from pyquaternion import Quaternion
from scipy.optimize import fsolve

# https://stackoverflow.com/questions/4870393/rotating-coordinate-system-via-a-quaternion
# https://kieranwynn.github.io/pyquaternion/

rEngine = 32;  # radius of the actuator engine mounts
hTopRing = 23; # axial (z) distance downwards between the pivot point and the engine top ring (bottom edge)
hEngine = 166.9; # axial (z) distance downwards between the pivot point and the engine bottom
lPivot = 106.9; # axial (z) distance downwards between the pivot point and the engine actuator mount points
hMount = 5; # axial (z) distance upwards between the pivot point and the stationary actuator mount points. 17-2: before cut = 63, after should = 63-14.8
rMount = 203; # radius of the stationary actuator mounts, r=120
aMax = 10*np.pi/180; # maximum gimbal angle in radians
lead = 2; # lead of ball screw in mm

# angles defined positive for movement in the y or x direction respectively
gimbal_angles = [10*pi/180,10*pi/180]  # a_outer, a_inner
#print("test inputs: a_outer = ", a_outer*180/pi, ", a_inner = ", a_inner*180/pi)

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
    def actuator_lengths(gimbal_angles):
        # Actuator 1 is in the first quadrant, 2 in the second

        # actuator thrust plate mount points
        a1_thrust_plate_mount = np.array([rMount*cos(pi/4),rMount*sin(pi/4),hMount])
        a2_thrust_plate_mount = np.array([rMount*cos(3*pi/4),rMount*sin(3*pi/4),hMount])

        # Non-rotated engine actuator mount points
        a1_engine_mount_original = np.array([rEngine*cos(pi/4),rEngine*sin(pi/4),-lPivot])
        a2_engine_mount_original = np.array([rEngine*cos(3*pi/4),rEngine*sin(3*pi/4),-lPivot])

        # rotated actuator mount points
        a1_engine_mount = TVCKinematics.rotate_gimbal_angles(gimbal_angles,a1_engine_mount_original)
        a2_engine_mount = TVCKinematics.rotate_gimbal_angles(gimbal_angles,a2_engine_mount_original)

        # required actuator lengths from desired position
        a1length = np.sqrt(np.sum((a1_engine_mount-a1_thrust_plate_mount)**2, axis=0))
        a2length = np.sqrt(np.sum((a2_engine_mount-a2_thrust_plate_mount)**2, axis=0))

        return [a1length,a2length]

    # calculate gimbal angles (outputs 1x2 np array) from given actuator lengths (forward kinematics)
    def gimbal_angles(alengths):

        def func(angles):
            lengths = TVCKinematics.actuator_lengths(angles[0],angles[1])
            return [lengths[0]-alengths[0],lengths[1]-alengths[1]]

        return fsolve(func,[0,0])
    
    def angle_between_vectors(u, v):
            dot_product = sum(i*j for i, j in zip(u, v))
            norm_u = sqrt(sum(i**2 for i in u))
            norm_v = sqrt(sum(i**2 for i in v))
            cos_theta = dot_product / (norm_u * norm_v)
            angle_rad = acos(cos_theta)
            return angle_rad

    # calculate xy component angles (angle between x perpendicular plane and thrust vector, vice versa for y) from gimbal angles
    def xy_component_angles(gimbal_angles):
        Tvec = TVCKinematics.rotate_gimbal_angles(gimbal_angles,np.array([0,0,-1]))
        xangle = pi/2-TVCKinematics.angle_between_vectors(Tvec, [1,0,0])
        yangle = pi/2-TVCKinematics.angle_between_vectors(Tvec, [0,1,0])
        return [xangle,yangle]
    
    # calculate gimbal angles from xy component angles
    def gimbal_angles_from_xy(xyangles):

        def func(gimbal_angles):
            xy = TVCKinematics.xy_component_angles(gimbal_angles)
            return [xy[0]-xyangles[0],xy[1]-xyangles[1]]

        return fsolve(func,xyangles)

    





    
