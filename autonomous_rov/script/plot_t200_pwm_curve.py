import pandas as pd
from scipy.optimize import curve_fit
import os
import numpy as np

def linear_eq(data, params):
    m,c = params
    x, y = data
    err = y - m*data + c
    return err

file_name = 't200.xls'
sheet_name = '16 V'
file_path = os.path.abspath(os.getcwd()) + '/data/' + file_name

print(file_path)
data = pd.read_excel(file_path, sheet_name=sheet_name)

# print(data.loc[data[' PWM (µs)'] >= 1500])
# print(data.loc[(data[' PWM (µs)'] >= 1500) & (data[' Force (Kg f)'] > 0)])
data_above_1500 = data.loc[(data[' PWM (µs)'] >= 1500) & (data[' Force (Kg f)'] > 0)]
print(data_above_1500)

curve_fit(linear_eq, data_above_1500[' PWM (µs)'].to_numpy(dtype=object), data_above_1500[' Force (Kg f)'].to_numpy(dtype=object))