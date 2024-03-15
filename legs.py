import trusspy as tp
import numpy as np
from sympy import *
from math import *

# Leg geometry - refer to notebook or slides

xu = 100 * 0.001
yu = 100 * 0.001
zu = -212 * 0.001

yg = -600 * 0.001
zg = -713 * 0.001

wu = 440 * 0.001

# Tip over analysis

g = 9.81
m = 150
v = 2
hcg = 1200 * 0.001
Ig = 2.7 * 10**10 # gmm^2
Ig = Ig / 1000 

r = sqrt(hcg**2 + 0.125 * (2*yg + wu)**2)

I0 = Ig + m*r**2

thetadot = m*v*hcg / (Ig + m*r**2)

factor = (I0*thetadot**2 + 2*m*g*hcg) / (2*m*g)

tip_factor = factor/r

#print(tip_factor)






# Stiffness analysis using trusspy

M = tp.Model(logfile=True)

#https://trusspy.readthedocs.io/en/latest/examples/eNTA-A/eNTA-A.html

with M.Nodes as MN:  # node creation
    MN.add_node(1, coord=(0, 0, 0))
    MN.add_node(2, coord=(xu, yu, zu))
    MN.add_node(3, coord=(-xu, yu, zu))
    MN.add_node(4, coord=(0, yg, zg))

element_type = 1  # truss
material_type = 1  # linear-elastic

youngs_modulus = 69 * 10**9
A_large = pi * 0.03 ** 2
A_small = pi * 0.01 ** 2

with M.Elements as ME: # link creation (not sure gpropr is yet)
    ME.add_element(1, conn=(1, 4), geometric_properties=[A_large])#, gprop=[0.75])
    ME.add_element(2, conn=(2, 4), geometric_properties=[A_small])#, gprop=[1])
    ME.add_element(3, conn=(3, 4), geometric_properties=[A_small])#, gprop=[0.5])

    ME.assign_etype("all", element_type)
    ME.assign_mtype("all", material_type)
    ME.assign_material("all", [youngs_modulus])

with M.Boundaries as MB: # fix nodes 1,2,3
    MB.add_bound_U(1, (0, 0, 0))
    MB.add_bound_U(2, (0, 0, 0))
    MB.add_bound_U(3, (0, 0, 0))

up_force = 1000 # this is not reflective of actual loading modes, it is just to find the vertical stiffness

with M.ExtForces as MF: # vertical force 
    MF.add_force(4, (0, 0, up_force))


M.Settings.dlpf = 0.005   # some settings to tweak around
M.Settings.du = 0.05
M.Settings.incs = 100
M.Settings.stepcontrol = True
M.Settings.maxfac = 4
M.Settings.ftol = 8
M.Settings.xtol = 8
M.Settings.nfev = 8
M.Settings.dxtol = 1.25

M.build()
M.run()


pinc = M.Settings.incs

fig, ax = M.plot_model(
    view="3d",
    contour="force",
    lim_scale=(-0.5, 0.5, -0.5, 0.5, -1, 0),
    force_scale=1,
    inc=pinc,
)
fig.savefig("loaded.png")

#'''
M.plot_movie(
    view="3d",
    contour="force",
    lim_scale=(-0.5, 0.5, -0.5, 0.5, -1, 0),  # 3D
    # lim_scale=-5, #XZ
    # lim_scale=(-4,4,-2,6), #XY
    # lim_scale=(-2,6,-2,6), #YZ
    #cbar_limits=[-0.3, 0.3],
    force_scale=1,
    incs=range(0, M.Settings.incs, 5),
)
#'''

# displacement of feet attachment

Disp = "Displacement Z"
fig, ax = M.plot_history(nodes=[4, 4], X=Disp, Y="Force Z")
fig.savefig("feet_Disp.png")
print("done")
