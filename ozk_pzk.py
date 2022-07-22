import matplotlib.pyplot as plt
import numpy as np

#length of the YOUBOT's links 
l = [0.033, 0.147, 0.155, 0.135 + 0.2175]

#from angle to radians 
def to_rad(angle):
    return angle * np.pi /180

#YOUBOT angle(rad) function
### time - time in f(t)
### omega - omega in sin(omega * t)
### ampl - Amplitude in A * sin(x)
### joint_num - number of youbot joint
def r(time, omega, ampl, joint_num):
    return np.sin(time * omega + np.pi/2) * ampl + thao_angles[joint_num]

#converting from YOUBOT angles(rad) to KINEMATIC model angles(rad)
def thao(r_array, angle_diff):
    r_temp = r_array.copy()
    for i in range(len(r_temp) - 1):
        r_temp[i] = r_temp[i] - angle_diff[i]
    return r_temp

#PZK for X
def x_f(angles):
    return np.cos(angles[1 -1]) * (l[0] + l[2] * np.sin(angles[2 -1]) + l[3] * np.sin(angles[2 -1] + angles[3-1]))

#PZK for Y
def y_f(angles):
    return np.sin(angles[1 -1]) * (l[0] + l[2] * np.sin(angles[2 -1]) + l[3] * np.sin(angles[2 -1] + angles[3-1]))

#PZK for Z
def z_f(angles):
    return (l[2] * np.cos(angles[2 -1]) + l[3] * np.cos(angles[2 -1] + angles[3-1]))

#OZK for T1
def thao_1(x, y, z, l):
    arra = []
    for xi, yi in np.c_[x, y]:
        if np.arctan2(yi,xi) < -1:
            arra.append(np.arctan2(yi,xi) + np.pi)
        else:
            arra.append(np.arctan2(yi,xi))
    return arra


#OZK for T1
def thao_2_helper(x, y, z, l):
    xp = x * np.cos(thao_1(x, y, z, l)) + y * np.sin(thao_1(x, y, z, l))
    x_l0 = []
    for i in np.c_[xp]:
        if i > 0:
            x_l0.append((i - l[0])[0])
        else:
            x_l0.append((i + l[0])[0])
    x_l0 = np.array(x_l0)
    p  = np.sqrt(np.square(x_l0) + np.square(z))
    b = (np.square(l[2]) + np.square(p) - np.square(l[3])) / (2 * l[2] * p)
    return np.arctan2(x_l0, z) - np.arctan2(np.sqrt(1-np.square(b)), b)

def thao_2(x, y, z, l):
    arra = []
    for xi, yi, zi in np.c_[x, y, z]:
        if thao_2_helper(xi, yi, zi, l) < -1:
            arra.append(thao_2_helper(xi, yi, zi, l)[0] + 2*np.pi)
        else:
            arra.append(thao_2_helper(xi, yi, zi, l)[0])
    return arra

#OZK for T1
def thao_3_helper(x, y, z, l):
    th_1 = thao_1(x, y, z, l)
    xp = x * np.cos(th_1) + y * np.sin(th_1)
    x_l0 = []
    for i, k in np.c_[xp, th_1]:
        if i > 0:
            if k < 0:
                x_l0.append((l[0] + i))
            else:
                if i < l[0]:
                    x_l0.append((l[0] - i))
                else:
                    x_l0.append((i - l[0]))
        else:
            if k > 0:
                x_l0.append((l[0] + i))
            else:
                if i > l[0]:
                    x_l0.append((-l[0] + i))
                else:
                    x_l0.append((i + l[0]))

    x_l0 = np.array(x_l0)

    p  = np.sqrt(np.square(x_l0) + np.square(z))

    a  = (np.square(l[2]) + np.square(l[3]) - np.square(p)) / (2 * l[2] * l[3])
    for i in np.c_[a]:
        if (i > 1):
            print(i)
    return np.pi - np.arctan2(np.sqrt(1-np.square(a)), a)

def thao_3(x, y, z, l):
    arra = []
    for xi, yi, zi in np.c_[x, y, z]:
        if thao_3_helper(xi, yi, zi, l) < 2.8:
            arra.append(thao_3_helper(xi, yi, zi, l)[0])
        else:
            arra.append(thao_3_helper(xi, yi, zi, l)[0])
    return arra


#GRAPHs
def graph3d(x, y, z):
    fig = plt.figure()
    ax = fig.add_subplot(projection='3d')

    ax.scatter(x, y, z)

    ax.set_xlabel('X Label')
    ax.set_ylabel('Y Label')
    ax.set_zlabel('Z Label')



#middle angles of YOUBOT
thao_angles = np.array([169, 65, 146, 102, 167])
thao_angles = to_rad(thao_angles)

#creating time in the UNIVERSE
time = np.arange(0, 200, 0.2)

#array of their values in time by R function
r_array = [r(time, 1.8, np.pi/4, 1 -1),
r(time, 1.8, np.pi/4, 2 -1),
r(time, 1.8, np.pi/4, 3 -1)]

#plt.plot(time, r_array[0])


#converting to KINEMATIC model
thao_array = thao(r_array, thao_angles)

#PZK coordinates
x = x_f(thao_array)
y = y_f(thao_array)
z = z_f(thao_array)

print(x[0])
print(y[0])
print(z[0])

#########START OZK############

thaos = [
    np.array(thao_1(x,y,z,l)),
    np.array(thao_2(x,y,z,l)),
    np.array(thao_3(x,y,z,l))
]


#2dGraph thao_array[n] with thaos[n]

plt.plot(time, thao_array[0])
#plt.plot(time, r_array[0])
plt.plot(time, thaos[0])
#plt.show()
#plt.plot(time, x)
#plt.plot(x, y)

x2 = x_f(thaos)
y2 = y_f(thaos)
z2 = z_f(thaos)

x_coor = np.array([-0.1])
y_coor = np.array([-0.1])
z_coor = np.array([-0.05])

thaos_coor = [
    np.array(thao_1(x_coor,y_coor,z_coor,l)),
    np.array(thao_2(x_coor,y_coor,z_coor,l)),
    np.array(thao_3(x_coor,y_coor,z_coor,l))
]

print(thaos_coor)

x_coor = x_f(thaos_coor)
y_coor = y_f(thaos_coor)
z_coor = z_f(thaos_coor)

print(x_coor)
print(y_coor)
print(z_coor)

#graph3d(x, y, z)
#graph3d(x2, y2, z2)
#plt.plot(time,z)
#plt.plot(time,z2)

plt.show()



##########END OZK#############

#GRAPH
#graph3d(x, y, z)


