#!/usr/bin/env python
import os
import math
import numpy as np
import pandas as pd 
import matplotlib.pyplot as plt
import sympy as sp
from Cubic_Trajectory import Rov_Trajectory 
#Estimating the heave from the depth measurements with an alpha-beta filter

def SM_Controller(desired_Trajectory, real_Trajectory, desired_speed, real_Speed, rho, K, phi):
    #Parameters : 
    global fs
    Dw = 5.2        #Linear dampping coef
    m = 11.5        #ROV mass (kg)
    x = np.array([0, 0])  #State vector ([depth, heave speed])
    
    #Errors :
    Z_tilde = desired_Trajectory - real_Trajectory  #Depth error
    W_tilde = desired_speed - real_Speed            #Speed error
    
    #SM surface : 
    S = rho[0] * Z_tilde + W_tilde
    #Command with chatering 
    f_z = -(m * rho[0] - Dw) * W_tilde - K * m * np.sign(S)

    #Command without chatering 
    """if (np.abs(S) < phi) : 
        f_z = -(m * rho[0] - Dw) * W_tilde - K * m * S / phi
    else :
        f_z = -(m * rho[0] - Dw) * W_tilde - K * m *np.sign(S / phi) """

    return f_z

####TEST######
