from cProfile import label
import numpy as np
import matplotlib.pyplot as plt
import pdb

import os
import pandas as pd

### Set your path to the folder containing the .csv files
file_name = 'depth_control_12v_kp_p_2_1.5_g_3_ki_0.005_no_wall_2.csv'
file_path = os.path.abspath(os.getcwd()) +"/filters/"+ file_name
print (file_path)

### Read the .csv file and make a df
df = pd.read_csv(file_path, index_col = None)

bias = -0.1
# print (bias)
Y = np.array(df['field.data'].tolist()) - bias
X = np.arange(len(Y))


## filter parameters
dt = 0.02 #50Hz data_read-control loop
xk_1 = 0
vk_1 = 0
ak_1 = 0

a = 0.45
b = 0.1
g = 0.0000

Y_f = np.zeros_like(Y)

for i in range(len(Y)):

    xm = Y[i]

    xk = xk_1 + (vk_1 * dt)
    vk = vk_1 + (ak_1 * dt)
    ak = (vk - vk_1) /dt

    rk = xm - xk

    xk += a*rk
    vk += (b*rk)/dt
    ak += (2*g*rk)/np.power(dt,2)

    xk_1 = xk
    vk_1 = vk
    ak_1 = ak

    Y_f[i] = xk

## filter parameters
dt = 0.02 #50Hz data_read-control loop
xk_1 = 0
vk_1 = 0
ak_1 = 0

a = 0.25
b = 0.025
g = 0.0001

Y_f2 = np.zeros_like(Y)

for i in range(len(Y)):

    xm = Y[i]

    xk = xk_1 + (vk_1 * dt)
    vk = vk_1 + (ak_1 * dt)
    ak = (vk - vk_1) /dt

    rk = xm - xk

    xk += a*rk
    vk += (b*rk)/dt
    ak += (2*g*rk)/np.power(dt,2)

    xk_1 = xk
    vk_1 = vk
    ak_1 = ak

    Y_f2[i] = xk

plt.plot( X, Y, linewidth=1.5, color='red', linestyle=":", label="raw data")
plt.plot( X, Y_f, linewidth=0.75, color='green', linestyle="-.", label=f"filtered data(a=0.45,b=0.1)")
plt.plot( X, Y_f2, linewidth=0.75, color='blue', linestyle="--", label=f"filtered data(a={a},b={b},g={g})")

### Generate the plot
plt.ylabel("Depth(m)")
plt.xlabel("Time Steps")
plt.legend(loc="best")
plt.axhline(-0.5,color='grey', label='Target depth') # y = 0
plt.grid()
plt.show()