# -*- coding: utf-8 -*-
"""
Created on Mon Mar 14 14:43:33 2022

@author: ryane
"""

import math
import numpy as np
import matplotlib.pyplot as plt
import pandas as pd 
from matplotlib.widgets import Cursor
import sympy as sp

#Import excell file  
xls = pd.ExcelFile('C:/Users/ryane/OneDrive/Desktop/STAGE/TP_BLUEROV_MIR/T200.xlsx')
#Show sheet names of excel file
print(xls.sheet_names)
#Read the data of sheet named "16V"
data_16V = pd.read_excel('C:/Users/ryane/OneDrive/Desktop/STAGE/TP_BLUEROV_MIR/T200.xlsx', sheet_name = '16 V')

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

#Question 2 : Interpolation 
#Calculate the slope of the curve of PWM  :
def linearFunction(x, y):
    m,b = np.polyfit(x, y, 1)
    print("m:", m, "b", b)
    return m*x+b

#positive PWM function 
PWM_neg = PWM_data[0:102]
Thrust_neg = Thrust_data[0:102]

PWM_neg_fun = linearFunction(Thrust_neg, PWM_neg)

#negative PWM function 
PWM_pos = PWM_data[102: :]
Thrust_pos = Thrust_data[102: :]
PWM_pos_fun = linearFunction(Thrust_pos, PWM_pos)

#Ploting : positive and negative PWM curve
plt.plot(Thrust_neg, PWM_neg_fun)
plt.plot(Thrust_pos, PWM_pos_fun )


###Question 3 : PI controller 
#Initialisation 

def PI_Controller(x_desired, x_real, K_P, K_I, step, I0):
    
    e = x_real - x_desired               #Error between the real and desired value 
    P = K_P * e                          #Proportional controller 
    I = I0 + K_I * e * step              #Integral controller
    Tau = P + I                          #Output of the PID controller 
    I0 = I                               #Update the initial value of integral controller 
    
    return Tau
    
def PI_Controller_With_Comp(x_desired, x_real, K_P, K_I, step, I0,g):
    
    e = x_real - x_desired               #Error between the real and desired value 
    P = K_P * e                          #Proportional controller 
    I = I0 + K_I * e * step              #Integral controller
    Tau = P + I + g                      #Output of the PID controller 
    I0 = I                               #Update the initial value of integral controller 
    
    return Tau



###############################PRACTICAL WORK 2##############################
#STEP 1 : ROV Trajectory : Cubic trajectory
def Rov_Trajectory(Z_init, Z_final, t, t_final) :
    a_2 = 3 * (Z_final - Z_init) / t_final**2       #Polynomial parameter 
    a_3 = -2 * (Z_final - Z_init) / t_final**3      #Polynomial parameter
    
    if (t < t_final) :
        Z_desired = Z_init + a_2 * t**2 +a_3 * t**3         #Desired depth trajectory(m)
        Zd_desired =  Z_init + 2 * a_2 * t + 3 * a_3 * t**2     #Desired vertical speed (m/s)
    elif (t >= t_final) :
        Z_desired = Z_final
        Zd_desired = 0
        
    return  Z_desired, Zd_desired

#Ploting : 
#Initialisation 
t_final = 20
Z_init = 0
Z_final = 1
Z_desired, Zd_desired = np.zeros((2, t_final))  
 
for t in range(t_final):
    Z_desired[t], Zd_desired[t] = Rov_Trajectory(Z_init , Z_final, t, t_final) 
  
print("Zd_desired", Z_desired.shape) 
t = np.arange(20)
plt.figure(2)
plt.plot(t, Z_desired, Zd_desired)
plt.xlabel("Time (s)")
plt.ylabel("Depth (m) or vertical speed (m/s)")
plt.title("curves of depth/ vertical speed")
#plt.legend("Z_desired", "Zd_desired")

#STEP 2 : Test The PI controller : 
    
    
#STEP 3 : Estimating the heave from the depth measurements with an alpha-beta filter

#Initialisation 
m = 11       #ROV Mass
D_w =  5.1   #Damping z
C0 = 1.2  
K = 3
phi = 0.1
step = 0.1
C = np.array([C0, 1])
Z_tilde = np.zeros((t_final))
W_tilde = np.zeros((t_final))
W_real = np.zeros((t_final))
Z_real = np.zeros((t_final))
Wd_tilde = np.zeros((t_final))
Zd_tilde = np.zeros((t_final))

S = np.zeros((t_final))
f_z = np.zeros((t_final))
Wd_real = np.zeros((t_final))

for t in range (t_final):
    #Error of depth 
    Z_tilde[t] = Z_real[t] - Z_desired[t] 
    W_tilde[t] = W_real[t] - Zd_desired[t] 
              
    #Sliding mode surface 
    S[t] = C[0] * Z_tilde[t] + W_tilde[t]
    
    #Commande 
    f_z[t] = -(m *C[0] - D_w) * W_tilde[t] - K * m * np.sign((S[t]))
    
    #Eliminating the chattering phenomenon
    f_z[t] = -(m *C[0] - D_w) * W_tilde[t] - K * m * np.arctan((S[t]))   
    if np.abs(C0 * Z_tilde[t] + W_tilde[t]) < 0 :
        f_z[t] = -(m * C0 - D_w) * W_tilde[t] - K * m * S[t] / phi
    
    else :
        f_z[t] = -(m * C0 - D_w) * W_tilde[t] - K * m * np.sign(S[t] / phi) 
    
    
    #Update the Rov model :
        
    Wd_real[t] = 1 / m * (f_z[t] - D_w * W_real[t])
    W_real [t] = W_real [t] + step *  Wd_real[t]
    Z_real[t] =  Z_real[t] + step * W_real [t]

plt.figure(3)    
plt.plot(W_real, "b",Z_real, "b")


    
    



