import matplotlib.pyplot as plt
import numpy as np


def thao(time):
    return np.sin(time*1.8 + np.pi/2) * np.pi/4 + 102 * np.pi / 180

def rot(r):
    return r - 102 * np.pi / 180

def x(vals):
    l = 217.5
    return l * np.sin(vals)

def y(vals):
    return 0

def z(vals):
    l = 217.5
    l0 = 147 + 155 + 135
    return l0 + l * np.cos(vals)

time = np.arange(0, 10000, 0.2)

vals = thao(time)

vals_youbot = rot(vals)

fig = plt.figure()
ax = fig.add_subplot(projection='3d')

x4 = x(vals_youbot)
y4 = y(vals_youbot)
z4 = z(vals_youbot)
ax.scatter(x4, y4, z4)

ax.set_xlabel('X Label')
ax.set_ylabel('Y Label')
ax.set_zlabel('Z Label')

fig1 = plt.figure()
ax1 = fig1.add_subplot()
ax1.plot(x4)


plt.show()