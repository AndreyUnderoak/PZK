import matplotlib.pyplot as plt
import numpy as np

def to_rad(angle):
    return angle * np.pi /180

thao_angles = np.array([169, 65, -146, 102, 167])
thao_angles = to_rad(thao_angles)

l = [0.033, 0.147, 0.155, 0.135, 0.2175]


def r(time, omega, ampl, joint_num):
    return np.sin(time * omega + np.pi/2) * ampl + thao_angles[joint_num]

def thao(r_array):
    for i in range(4):
        r_array[i] = r_array[i] - thao_angles[i]
    return r_array

def x_f(angles):
    return np.cos(angles[1 -1]) * (l[0] + l[2] * np.sin(angles[2 -1]) + l[3] * np.sin(angles[2 -1] - angles[3-1]) + l[4] * np.sin(angles[2 -1] - angles[3-1] + angles[4 -1]))

def y_f(angles):
    return - np.sin(angles[1 -1]) * (l[0] + l[2] * np.sin(angles[2 -1]) + l[3] * np.sin(angles[2 -1] - angles[3-1]) + l[4] * np.sin(angles[2 -1] - angles[3-1] + angles[4 -1]))

def z_f(angles):
    return (l[1] + l[2] * np.cos(angles[2 -1]) + l[3] * np.cos(angles[2 -1] - angles[3-1]) + l[4] * np.cos(angles[2 -1] - angles[3-1] + angles[4 -1]))
 

time = np.arange(0, 200, 0.2)

r_array = [r(time, 1.8, np.pi/4, 1 -1),
r(time, 1.8, np.pi/4, 2 -1),
r(time, 1.8, np.pi/4, 3 -1),
r(time, 1.8, np.pi/4, 4 -1)]

thao_array = thao(r_array)

x = x_f(thao_array)
y = y_f(thao_array)
z = z_f(thao_array)

fig = plt.figure()
ax = fig.add_subplot(projection='3d')

ax.scatter(x, y, z)

ax.set_xlabel('X Label')
ax.set_ylabel('Y Label')
ax.set_zlabel('Z Label')

plt.show()
