import matplotlib.pyplot as plt
import numpy as np

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
    
    def a_matrix(self, theta_array, link_num):
        return np.array([  [np.cos(theta_array[link_num] + self.links[link_num].offset), - np.sin(theta_array[link_num] + self.links[link_num].offset) * np.cos(self.links[link_num].alpha), np.sin(theta_array[link_num] + self.links[link_num].offset) * np.sin(self.links[link_num].alpha)  , self.links[link_num].a*np.cos(theta_array[link_num] + self.links[link_num].offset)],
                           [np.sin(theta_array[link_num] + self.links[link_num].offset), np.cos(theta_array[link_num] + self.links[link_num].offset) * np.cos(self.links[link_num].alpha)  , - np.cos(theta_array[link_num] + self.links[link_num].offset) * np.sin(self.links[link_num].alpha), self.links[link_num].a*np.sin(theta_array[link_num] + self.links[link_num].offset)],
                           [0                                                          , np.sin(self.links[link_num].alpha)                                                                , np.cos(self.links[link_num].alpha)                                                                , self.links[link_num].d                                                            ],
                           [0                                                          , 0                                                                                                 , 0                                                                                                 , 1                                                                                 ]])

    def h_matrix(self, theta_array):
        a = self.a_matrix(theta_array, 0)

        for i in range(1,len(self.links)):
            a = np.dot(a , self.a_matrix(theta_array, i))

        return a
        
    def get_coors(self, theta_array):
        a = self.h_matrix(theta_array)
        return np.array([a[0][3], a[1][3], a[2][3]])
    
    def get_rot_matrix(self, theta_array):
        a = self.h_matrix(theta_array)
        return np.array([[a[0][0],a[0][1],a[0][2]],
                         [a[1][0],a[1][1],a[1][2]],
                         [a[2][0],a[2][1],a[2][2]]])

youbot_dh = DH_param([  Link(offset = 0,        d = -links_length[1],  a = links_length[0], alpha = np.pi/2),
                        Link(offset = -np.pi/2, d = 0,                 a = links_length[2], alpha = -np.pi),
                        Link(offset = 0,        d = 0,                 a = links_length[3], alpha = np.pi),
                        Link(offset = -np.pi/2, d = 0,                 a = 0,               alpha = np.pi/2),
                        Link(offset = 0,        d = -links_length[4],  a = 0,               alpha = 0)])

print(youbot_dh.get_coors(np.array([0,0,0,0,0])))
print(youbot_dh.get_rot_matrix(np.array([0,0,0,0,0])))
#################FKP END#################
