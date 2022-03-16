import pandas as pd
import matplotlib as plot

data = pd.read_excel (r'xlsdata.xlsx')
df = pd.DataFrame(data, columns= [' PWM (µs)', ' Force (Kg f)'])
force = df.loc[:," Force (Kg f)"]
pwm = df.loc[:," PWM (µs)"]
print (force, pwm)