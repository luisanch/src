import pandas as pd
import matplotlib.pyplot as plt
import numpy as np

data = pd.read_excel (r'xlsdata.xlsx')
df = pd.DataFrame(data, columns= [' PWM (µs)', ' Force (Kg f)'])
dfnp = df.to_numpy()
plt.plot(dfnp[:,1], dfnp[:,0])
plt.grid()
plt.show()
 