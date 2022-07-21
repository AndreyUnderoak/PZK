import pandas as pd
import matplotlib.pyplot as plt
import numpy as np

headers = ['TIME','GOAL','REAL']
df = pd.read_csv('angles.csv') 

print(df)

time = df['TIME']
r_goal = df['GOAL']
r_real = df['REAL']
error = r_goal - r_real

plt.plot(time,r_goal)
plt.plot(time, r_real)
plt.plot(time, error)
plt.show()
