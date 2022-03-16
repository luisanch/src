from numpy.core.defchararray import array
import pandas as pd
from scipy.optimize import curve_fit
import os
import numpy as np
import matplotlib.pyplot as plt

def linear_eq(x, m):
    # print(prams)
    # m,c = params
    # x, y = data
    return m*x + 1532
    

file_name = 't200.xls'
sheet_name = '16 V'
file_path = os.path.abspath(os.getcwd()) + '/data/' + file_name

print(file_path)
data = pd.read_excel(file_path, sheet_name=sheet_name)

# print(data.loc[data[' PWM (µs)'] >= 1500])
# print(data.loc[(data[' PWM (µs)'] >= 1500) & (data[' Force (Kg f)'] > 0)])
data_above_1500 = data.loc[(data[' PWM (µs)'] >= 1500) & (data[' Force (Kg f)'] > 0)]
print(data_above_1500)
y = np.array(data_above_1500[' PWM (µs)'])
x = np.array(data_above_1500[' Force (Kg f)'])
result = curve_fit(linear_eq, x, y, p0=[0])
y_all = np.array(data[' PWM (µs)'])
x_all = np.array(data[' Force (Kg f)'])
print(result)
plt.plot(x, linear_eq(x, result[0][0]))
plt.plot(x_all, y_all)
plt.show()

