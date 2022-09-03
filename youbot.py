import matplotlib.pyplot as plt
import numpy as np
import roboticstoolbox as rtb

np.set_printoptions(precision=4, suppress=True)

class Youbot :
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
    
    #length of the YOUBOT's links 
    links_length = [0.033, 0.147, 0.155, 0.135, 0.2175]

    #YOUBOT angles
    youbot_angles = np.radians(np.array([169, 65, -146, 102, 167]))

    class Joint:
        def __init__(self, min, max):
            self.min = min
            self.max = max
        
        def in_range(self, youbot_angle):
            if self.min < youbot_angle and self.max > youbot_angle :
                return True
            else :
                return False
            
    joints = np.array([
        Joint(np.radians(-169), np.radians(169)),
        Joint(np.radians(-65 ), np.radians(90 )),
        Joint(np.radians(-151), np.radians(146)),
        Joint(np.radians(-102), np.radians(102)),
        Joint(np.radians(-167), np.radians(167))
    ])
    
    class Link:
        def __init__(self, flip, offset, d, a, alpha):
            self.flip = flip
            self.offset = offset
            self.d      = d
            self.a      = a
            self.alpha  = alpha
            
    links = np.array([
        Link(flip = True,  offset = 0,        d = links_length[1],   a = links_length[0], alpha = -np.pi/2),
        Link(flip = False, offset = -np.pi/2, d = 0,                 a = links_length[2], alpha = 0),
        Link(flip = False, offset = 0,        d = 0,                 a = links_length[3], alpha = 0),
        Link(flip = False, offset = np.pi/2, d = 0,                 a = 0,               alpha = -np.pi/2),
        Link(flip = False, offset = 0,        d = -links_length[4],  a = 0,               alpha = 0)
    ])

    def __init__(self):
        self.view_model_init(self.links)

    def view_model_init(self, links):
        temp = np.zeros(len(links), rtb.RevoluteDH)
        for i in range(len(links)):
            temp[i] = rtb.RevoluteDH(d=links[i].d, a=links[i].a, alpha=links[i].alpha, offset = links[i].offset, flip=links[i].flip)
        
        self.view_model = rtb.DHRobot(temp.tolist(), name="YOUBOT")
        #self.view_model.plot(self.view_model.q, block=True)

    ################FKP START################
    def a_matrix(self, theta_array, link_num):
        return np.array([  [np.cos(theta_array[link_num] + self.links[link_num].offset), - np.sin(theta_array[link_num] + self.links[link_num].offset) * np.cos(self.links[link_num].alpha), np.sin(theta_array[link_num] + self.links[link_num].offset) * np.sin(self.links[link_num].alpha)  , self.links[link_num].a*np.cos(theta_array[link_num] + self.links[link_num].offset)],
                           [np.sin(theta_array[link_num] + self.links[link_num].offset), np.cos(theta_array[link_num] + self.links[link_num].offset) * np.cos(self.links[link_num].alpha)  , - np.cos(theta_array[link_num] + self.links[link_num].offset) * np.sin(self.links[link_num].alpha), self.links[link_num].a*np.sin(theta_array[link_num] + self.links[link_num].offset)],
                           [0                                                          , np.sin(self.links[link_num].alpha)                                                                , np.cos(self.links[link_num].alpha)                                                                , self.links[link_num].d                                                            ],
                           [0                                                          , 0                                                                                                 , 0                                                                                                 , 1                                                                                 ]])

    def h_matrix(self, theta_array, start, finish):
        a = self.a_matrix( - theta_array, start)

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
    #################FKP END#################
    ###############IKP START#################
    #OZK for T1
    ### x, y, z - axes
    ### l - lengthes of the links
    ### conf - configurations
    ##### conf = 1 - y > 0
    ##### conf = 2 - y < 0
    def t_1(self, x, y, conf_t_1):
        if conf_t_1 == 1 :
            return np.arctan2(y,x)
        if conf_t_1 == 2 :
            return np.arctan2(y,x) - np.pi 
    
    #OZK for T3
    def t_3(self, x, y, z, l, conf_t_1, conf_t_3):
        t_1_temp = self.t_1(x, y, conf_t_1)
        xp = x * np.cos(t_1_temp) + y * np.sin(t_1_temp)

        p_square = (xp - l[0])**2 + (z-l[1])**2
        
        cosT_3  = - (l[2]**2 + l[3]**2 - p_square) / (2 * l[2] * l[3])

        # IF NOT AN ARRAY
        if(type(cosT_3) == np.float64):
            if np.abs(cosT_3) > 1.0001 :
                raise Exception("OUT OF LINKS RANGE")
            if np.abs(cosT_3) > 1 :
                cosT_3 = np.round(cosT_3, 3)
        
        # IF ARRAY
        else:
            for i in range(len(cosT_3)):
                if np.abs(cosT_3[i]) > 1.0001 :
                    raise Exception("OUT OF LINKS RANGE")
                if np.abs(cosT_3[i]) > 1 :
                    cosT_3[i] = np.round(cosT_3[i], 3)
                    
        if conf_t_1 == 1:
            if conf_t_3 == 1 :
                return np.arctan2(np.sqrt(1 - cosT_3**2), cosT_3)
            if conf_t_3 == 2 :
                return - np.arctan2(np.sqrt(1 - cosT_3**2), cosT_3)
        else:
            if conf_t_3 == 2 :
                return np.arctan2(np.sqrt(1 - cosT_3**2), cosT_3)
            if conf_t_3 == 1 :
                return - np.arctan2(np.sqrt(1 - cosT_3**2), cosT_3)

    #OZK for T2
    def t_2(self, x, y, z, l, conf_t_1, conf_t_3):
        t_1_temp = self.t_1(x, y, conf_t_1)
        xp = x * np.cos(t_1_temp) + y * np.sin(t_1_temp)

        t_3_temp = self.t_3(x, y, z, l, conf_t_1, conf_t_3)
        beta = np.arctan2(l[3]*np.sin(t_3_temp), l[2]+l[3]*np.cos(t_3_temp))

        return np.arctan2(xp - l[0], z - l[1]) - beta
    
    def orientation(self, orientation_angle, theta_1, theta_5, conf_t_1):
        th1 = -theta_1
        if conf_t_1 == 2:
            th1 += np.pi

        m1 = np.zeros([3,3]); m2 = m1.copy(); m3 = m1.copy()

        m1[1][1] = np.cos(th1) 
        m1[0][0] = np.cos(th1)
        m1[0][1] = - np.sin(th1)
        m1[1][0] = np.sin(th1)
        m1[2][2] = 1

        m2[2][2] = np.cos(orientation_angle - np.pi)
        m2[0][0] = np.cos(orientation_angle - np.pi)
        m2[0][2] = np.sin(orientation_angle - np.pi)
        m2[2][0] = - np.sin(orientation_angle - np.pi)
        m2[1][1] = 1
        
        if conf_t_1 == 2:
            theta_5-=np.pi
            
        m3[1][1] = np.cos(theta_5) 
        m3[0][0] = np.cos(theta_5)
        m3[0][1] = - np.sin(theta_5)
        m3[1][0] = np.sin(theta_5)
        m3[2][2] = 1
        
        m1 = np.dot(m1,m2)
        m1 = np.dot(m1,m3)
        
        return m1

    def inverse_position(self, coordinates, theta_array, conf_t_1, conf_t_3):
        theta_array = np.append(theta_array, np.array
                ([
                    self.t_2(coordinates[0], coordinates[1], coordinates[2], self.links_length, conf_t_1, conf_t_3),
                    self.t_3(coordinates[0], coordinates[1], coordinates[2], self.links_length, conf_t_1, conf_t_3)
                ]))
        return theta_array

    def inverse_orientation(self, theta_array, r05):
        r03 = self.get_rot_matrix(theta_array, 0, 3)
        tr03 = np.transpose(r03)

        r35 = np.dot(tr03, r05)

        theta_array = np.append(theta_array, [
            np.arctan2(r35[0][2], -r35[1][2]) + np.pi/2,
            np.arctan2(r35[2][0], r35[2][1])
        ])

        return theta_array

    def inverse_get_theta_array(self, coordinates, conf_t_1, conf_t_3, orientation_angle = 0, ee_angle = 0):

        theta_array = np.array([- self.t_1(coordinates[0], coordinates[1], conf_t_1)])

        r05 = self.orientation(orientation_angle, theta_array[0], ee_angle, conf_t_1)

        #shift to p coordinates
        p = coordinates - np.dot(r05, np.array([0, 0, -self.links_length[4]]))


        theta_array = self.inverse_position(p, theta_array, conf_t_1, conf_t_3)
        theta_array = self.inverse_orientation(theta_array, r05)
        
        for i in range(len(theta_array)):
            if not self.joints[i].in_range(theta_array[i]) :
                raise Exception('OUT OF JOINT RANGE theta_array =',theta_array)
        
        return theta_array
    ################IKP END##################