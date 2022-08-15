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
def r_to_t(r_array, angle_diff):
    r_temp = r_array.copy()
    for i in range(len(r_temp) - 1):
        r_temp[i] -= angle_diff[i]
    return r_temp

def t_to_r(t_array, angle_diff):
    t_temp = t_array.copy()
    for i in range(len(t_temp) - 1):
        t_temp[i] += angle_diff[i]
    return t_temp

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
### x, y, z - axes
### l - lengthes of the links
### conf - configurations
##### conf = 1 - y > 0
##### conf = 2 - y < 0
def t_1(x, y, conf_t_1):
    if conf_t_1 == 1 :
        return np.arctan2(y,x)
    if conf_t_1 == 2 :
        return np.arctan2(y,x) - np.pi 

#OZK for T3
def t_3(x, y, z, l, conf_t_1, conf_t_3):
    t_1_temp = t_1(x, y, conf_t_1)
    xp = x * np.cos(t_1_temp) + y * np.sin(t_1_temp)

    p_square = (xp - l[0])**2 + z**2
    
    cosT_3  = - (l[2]**2 + l[3]**2 - p_square) / (2 * l[2] * l[3])

    for i in np.c_[cosT_3]:
        if np.abs(i) > 1 :
            raise Exception("OUT OF LINKS RANGE")

    if conf_t_3 == 1 :
        return np.arctan2(np.sqrt(1 - cosT_3**2), cosT_3)
    if conf_t_3 == 2 :
        return - np.arctan2(np.sqrt(1 - cosT_3**2), cosT_3)

#OZK for T2
def t_2(x, y, z, l, conf_t_1, conf_t_3):
    t_1_temp = t_1(x, y, conf_t_1)
    xp = x * np.cos(t_1_temp) + y * np.sin(t_1_temp)

    t_3_temp = t_3(x, y, z, l, conf_t_1, conf_t_3)
    beta = np.arctan2(l[3]*np.sin(t_3_temp), l[2]+l[3]*np.cos(t_3_temp))

    return np.arctan2(xp - l[0], z) - beta


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
thao_array = r_to_t(r_array, thao_angles)

#PZK coordinates
x = x_f(thao_array)
y = y_f(thao_array)
z = z_f(thao_array)


#########START OZK############
# try :
#     thaos = [
#         np.array(thao_1(x,y,z,l)),
#         np.array(thao_2(x,y,z,l)),
#         np.array(thao_3(x,y,z,l))
#     ]
#     plt.plot(time, thao_array[0])
#     plt.plot(time, thaos[0])
#     plt.show()
#     x2 = x_f(thaos)
#     y2 = y_f(thaos)
#     z2 = z_f(thaos)
#     graph3d(x, y, z)
#     graph3d(x2, y2, z2)
#     plt.show()

# except:
#     print("OUT OF LINKS RANGE")


################TEST################
t_1_conf = 2
t_3_conf = 1

for i in range(1, 3):
    for j in range(1, 3):
        t_1_conf = i
        t_3_conf = j
        print('conf ({}, {})'.format(i, j))

        x_coor = np.array([0.2])
        y_coor = np.array([0.2])
        z_coor = np.array([0.2])

        try :
            t_array = [
                np.array(t_1(x_coor,y_coor, t_1_conf)),
                np.array(t_2(x_coor,y_coor,z_coor,l, t_1_conf, t_3_conf)),
                np.array(t_3(x_coor,y_coor,z_coor,l, t_1_conf, t_3_conf))
            ]
            x_coor = x_f(t_array)
            y_coor = y_f(t_array)
            z_coor = z_f(t_array)

            print(t_array[0])
            print(t_array[1])
            print(t_array[2])

            print("------------")

            print(x_coor)
            print(y_coor)
            print(z_coor)

            print('------------------')

        except:
            print("OUT OF LINKS RANGE")




##########END OZK#############
###########PLANER#############

########NET PLANERA###########

########END PLANER###########
