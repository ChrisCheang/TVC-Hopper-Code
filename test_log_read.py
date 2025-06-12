import pandas as pd
import matplotlib.pyplot as plt
from math import *
import numpy as np
import sympy as smp
from sympy import *

# Load CSV file
csv_file = 'TEST_LOG_2025-06-11_23_17_05.csv'  # Change to your filename
df = pd.read_csv(csv_file)


def Derivative(x, xn, yn, k):
    xi = np.where(xn == x)[0][0]

    def Solver(x, xn, yn, k, forwards):
        h = xn[1] - xn[0]
        xi = np.where(xn == x)[0][0]
        if k == 1:
            return (yn[xi + forwards] - yn[xi + forwards - 1]) / h
        else:
            a = Solver(x + h * forwards, xn, yn, k - 1, forwards=forwards)
            b = Solver(x + h * forwards - h, xn, yn, k - 1, forwards=forwards)
            return (a - b) / h

    if xi < k:
        return Solver(x, xn, yn, k, forwards=True)
    else:  # default = backwards: more realistic for this situation as the future is not known
        return Solver(x, xn, yn, k, forwards=False)



motor0_speeds = [Derivative(df["Time_(s)"][i],df["Time_(s)"],df["Turns_M0"],1) for i in range(len(df["Time_(s)"]))]
motor1_speeds = [Derivative(df["Time_(s)"][i],df["Time_(s)"],df["Turns_M1"],1) for i in range(len(df["Time_(s)"]))]

motor0_accels = [Derivative(df["Time_(s)"][i],df["Time_(s)"],motor0_speeds,1) for i in range(len(df["Time_(s)"]))]
motor1_accels = [Derivative(df["Time_(s)"][i],df["Time_(s)"],motor1_speeds,1) for i in range(len(df["Time_(s)"]))]



plt.xlabel("Time (s)")       # X-axis title


plt.plot(df["Time_(s)"],df["Turns_M0"])
plt.plot(df["Time_(s)"],df["Target_M0"])
plt.plot(df["Time_(s)"],df["Turns_M1"])
plt.plot(df["Time_(s)"],df["Target_M1"])

plt.ylabel("Turns")       # Y-axis title
plt.title("Target vs Actual Position")        # Plot title

plt.minorticks_on()
plt.grid(which='major', linestyle='-', linewidth='0.5', color='black')
plt.grid(which='minor', linestyle=':', linewidth='0.5', color='gray')
plt.show()

plt.plot(df["Time_(s)"],df["Current_M0"])
plt.plot(df["Time_(s)"],df["Current_M1"])

plt.ylabel("Current")       # Y-axis title
plt.title("M0 vs M1 Current")        # Plot title

plt.minorticks_on()
plt.grid(which='major', linestyle='-', linewidth='0.5', color='black')
plt.grid(which='minor', linestyle=':', linewidth='0.5', color='gray')
plt.show()

plt.plot(df["Time_(s)"],motor0_speeds)
plt.plot(df["Time_(s)"],motor1_speeds)

plt.ylabel("RPS")       # Y-axis title
plt.title("M0 vs M1 Speed")        # Plot title

plt.minorticks_on()
plt.grid(which='major', linestyle='-', linewidth='0.5', color='black')
plt.grid(which='minor', linestyle=':', linewidth='0.5', color='gray')
plt.show()

plt.plot(df["Time_(s)"],motor0_accels)
plt.plot(df["Time_(s)"],motor1_accels)

plt.ylabel("RPS^2")       # Y-axis title
plt.title("M0 vs M1 Accel")        # Plot title

plt.minorticks_on()
plt.grid(which='major', linestyle='-', linewidth='0.5', color='black')
plt.grid(which='minor', linestyle=':', linewidth='0.5', color='gray')
plt.show()
