#!/usr/bin/env python
from numpy.core.defchararray import array
import os
import math
import numpy as np
import pandas as pd 
import matplotlib.pyplot as plt
import sympy as sp


#Read the data of sheet named "16V"
data_16V = pd.read_excel('/home/rayane/catkin_ws/src/src-main/autonomous_rov/data/T200.xls', sheet_name = '16 V')

###Question 1 : Plot the PWM curve (in µs, vertically) as a function of the desired thrust (in Newton, horizontally) of 16V

PWM_data = np.array(data_16V[' PWM (µs)'])
Thrust_data_kgf = np.array(data_16V[' Force (Kg f)'])

#Convert thrust data from kgf to N :
Thrust_data = np.zeros((Thrust_data_kgf.shape[0]))
for i in range(Thrust_data_kgf.shape[0]) :
    Thrust_data[i] = Thrust_data_kgf [i] * 10

#Ploting :
plt.plot(Thrust_data, PWM_data)
plt.xlabel("Thrust (F)")
plt.ylabel("PWM (us)")
plt.title("curve of PWM as a function of the desired thrust")


###Question 2 : Interpolation 

#Calculate the slope of the curve of PWM  :
def linearFunction(x, y):
    m,b = np.polyfit(x, y, 1)
    print("m:", m, "b", b)
    return m*x+b
#negative PWM function 
PWM_neg = PWM_data[0:102]
Thrust_neg = Thrust_data[0:102]
PWM_neg_fun = linearFunction(Thrust_neg, PWM_neg)



#positive PWM function 
PWM_pos = PWM_data[102: :]
Thrust_pos = Thrust_data[102: :]
PWM_pos_fun = linearFunction(Thrust_pos, PWM_pos)

#Ploting : positive and negative PWM curve
plt.plot(Thrust_neg, PWM_neg_fun)
plt.plot(Thrust_pos, PWM_pos_fun )
plt.show()