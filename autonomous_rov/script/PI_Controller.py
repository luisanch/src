#!/usr/bin/env python
import os
import math
import numpy as np
import pandas as pd 
import matplotlib.pyplot as plt
import sympy as sp

def PI_Controller(x_desired, x_real, K_P, K_I, step, I0):
    
    e = x_real - x_desired               #Error between the real and desired value 
    P = K_P * e                          #Proportional controller 
    I = I0 + K_I * e * step              #Integral controller
    Tau = P + I                          #Output of the PID controller 
    I0 = I                               #Update the initial value of integral controller 
    
    return Tau
    
def PI_Controller_With_Comp(x_desired, x_real, K_P, K_I, step, I0,flotability):
    
    e = x_real - x_desired               #Error between the real and desired value 
    P = K_P * e                          #Proportional controller 
    I = I0 + K_I * e * step              #Integral controller
    PI_Controller = P + I + flotability                     #Output of the PID controller 
    I0 = I                               #Update the initial value of integral controller 
    
    return PI_Controller 