from ctypes import sizeof
import pandas as pd 
import numpy as np 
import matplotlib.pyplot as plt
import os

namespace = 'br5'
filename = "_slash_" + namespace + "_slash_mavros_slash_imu_slash_water_pressure.csv"
file_path = os.path.abspath(os.getcwd())  
data = pd.read_csv(file_path + '/data_measurements/' + filename)
print(data)
data_timestamp = data.loc[:,'rosbagTimestamp']
data_pressure = data.loc[:,'fluid_pressure']

plt.plot(data_timestamp, data_pressure)
plt.show()

#print(data_timestamp.size, data_pressure.size)