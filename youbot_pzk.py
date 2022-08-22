import matplotlib.pyplot as plt
import numpy as np
import roboticstoolbox as rtb
import spatialmath as spm

#length of the YOUBOT's links 
links_length = [0.033, 0.147, 0.155, 0.135, 0.2175]

#YOUBOT angles
youbot_angles = np.radians(np.array([169, 65, -146, 102, 167]))

time = np.arange(0, 200, 0.2)

#YOUBOT angle(rad) function
### time - time in f(t)
### omega - omega in sin(omega * t)
### ampl - Amplitude in A * sin(x)
### joint_num - number of youbot joint
def r(time, omega, ampl):
    return np.sin(time * omega + np.pi/2) * ampl

#converting from YOUBOT angles(rad) to KINEMATIC model angles(rad)
def youbot_to_theta(r_array, angle_diff):
    r_temp = r_array.copy()
    for i in range(len(r_temp) - 1):
        r_temp[i] -= angle_diff[i]
    return r_temp

def theta_to_youbot(t_array, angle_diff):
    t_temp = t_array.copy()
    for i in range(len(t_temp) - 1):
        t_temp[i] += angle_diff[i]
    return t_temp

################FKP START################
class Link:
    def __init__(self, offset, d, a, alpha):
        self.offset = offset
        self.d      = d
        self.a      = a
        self.alpha  = alpha

class DH_param :
    def __init__(self, links):
        self.links = links
        self.view_model_init(links)

    def view_model_init(self, links):
        temp = np.zeros(len(links), rtb.RevoluteDH)
        for i in range(len(links)):
            temp[i] = rtb.RevoluteDH(d=links[i].d, a=links[i].a, alpha=links[i].alpha, offset = links[i].offset)
        
        self.view_model = rtb.DHRobot(temp.tolist(), base = spm.SE3.Rx(np.pi), name="YOUBOT")
        self.view_model.plot(self.view_model.q, block=True)

    def a_matrix(self, theta_array, link_num):
        return np.array([  [np.cos(theta_array[link_num] + self.links[link_num].offset), - np.sin(theta_array[link_num] + self.links[link_num].offset) * np.cos(self.links[link_num].alpha), np.sin(theta_array[link_num] + self.links[link_num].offset) * np.sin(self.links[link_num].alpha)  , self.links[link_num].a*np.cos(theta_array[link_num] + self.links[link_num].offset)],
                           [np.sin(theta_array[link_num] + self.links[link_num].offset), np.cos(theta_array[link_num] + self.links[link_num].offset) * np.cos(self.links[link_num].alpha)  , - np.cos(theta_array[link_num] + self.links[link_num].offset) * np.sin(self.links[link_num].alpha), self.links[link_num].a*np.sin(theta_array[link_num] + self.links[link_num].offset)],
                           [0                                                          , np.sin(self.links[link_num].alpha)                                                                , np.cos(self.links[link_num].alpha)                                                                , self.links[link_num].d                                                            ],
                           [0                                                          , 0                                                                                                 , 0                                                                                                 , 1                                                                                 ]])

    def h_matrix(self, theta_array, start, finish):
        a = self.a_matrix(theta_array, start)

        for i in range(start + 1, finish):
            a = np.dot(a , self.a_matrix(theta_array, i))

        return a
        
    def get_end_effector_coors(self, theta_array):
        a = self.h_matrix(theta_array, 0, 5)
        return np.array([a[0][3], a[1][3], a[2][3]])
    
    def get_rot_matrix(self, theta_array, start, finish):
        a = self.h_matrix(theta_array, start, finish)
        return np.array([[a[0][0],a[0][1],a[0][2]],
                         [a[1][0],a[1][1],a[1][2]],
                         [a[2][0],a[2][1],a[2][2]]])

youbot_dh = DH_param([  Link(offset = 0,        d = -links_length[1],  a = links_length[0], alpha = np.pi/2),
                        Link(offset = -np.pi/2, d = 0,                 a = links_length[2], alpha = -np.pi),
                        Link(offset = 0,        d = 0,                 a = links_length[3], alpha = np.pi),
                        Link(offset = -np.pi/2, d = 0,                 a = 0,               alpha = np.pi/2),
                        Link(offset = 0,        d = -links_length[4],  a = 0,               alpha = 0)])


print(youbot_dh.get_end_effector_coors([0,0,0,0,0]))
#################FKP END#################

###############IKP START#################

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


#ORIENTATION
angle_const = np.pi/2

r05 = np.zeros((3,3))
r05[1][1] = 1
r05[0][0] = r05[2][2] = np.cos(angle_const)
r05[2][0] = - np.sin(angle_const)
r05[0][2] = np.sin(angle_const)

#GOAL coordinates
x = 0.3
y = 0
z = 0.2
coordinates = np.array([[x,y,z]])
coordinates = np.transpose(coordinates)

#shift to p coordinates
p = coordinates - np.dot(r05, np.array([[0], [0], [links_length[4]]]))

p = np.transpose(p)[0]

conf_t_1 = 2
conf_t_3 = 1

theta_array = np.array([
    t_1(p[0], p[1], conf_t_1),
    t_2(p[0], p[1], p[2], links_length, conf_t_1, conf_t_3),
    t_3(p[0], p[1], p[2], links_length, conf_t_1, conf_t_3)
])


r03 = youbot_dh.get_rot_matrix(theta_array, 0, 3)
tr03 = np.transpose(r03)

r35 = np.dot(tr03, r05)

theta_array = np.append(theta_array, [
    np.arctan2(r35[0][2], -r35[1][2]) + np.pi/2,
    np.arctan2(r35[2][0], r35[2][1])
])

print(theta_array)

print(youbot_dh.get_end_effector_coors(theta_array))

# h = youbot_dh.view_model.fkine([theta_1, theta_2, theta_3, theta_4, theta_5])
# youbot_dh.view_model.plot([theta_1, theta_2, theta_3, theta_4, theta_5], block=True)
#plt.show()
################IKP END##################
