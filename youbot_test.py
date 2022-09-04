import pytest
import youbot
import numpy as np

class Test_class:
    youbot = youbot.Youbot()

    #only 11 configuration
    def test_inverse_t11(self):
        theta_for_test = np.array([
            [0,        np.pi/3, np.pi/4,  np.pi/2, 0], #t11 joint | link | link
            [0,        np.pi/3, np.pi/4,  np.pi/4, 0]  #t11 joint | link | link
        ])
        for i in range(len(theta_for_test)):
            ee_orientation = theta_for_test[i][1] + theta_for_test[i][2] + theta_for_test[i][3]
            coordinates = self.youbot.get_end_effector_coors(theta_for_test[i])

            assert self.youbot.get_end_effector_coors(
                self.youbot.inverse_get_theta_array(
                coordinates, 
                conf_t_1 = 1, 
                conf_t_3 = 1, 
                orientation_angle = np.abs(ee_orientation))).all() == coordinates.all()
            with pytest.raises(Exception):
                self.youbot.get_end_effector_coors(
                self.youbot.inverse_get_theta_array(
                coordinates, 
                conf_t_1 = 1, 
                conf_t_3 = 2, 
                orientation_angle = np.abs(ee_orientation)))
            for conf_3 in range(1, 3):
                with pytest.raises(Exception):
                    self.youbot.get_end_effector_coors(
                    self.youbot.inverse_get_theta_array(
                    coordinates, 
                    conf_t_1 = 2, 
                    conf_t_3 = conf_3, 
                    orientation_angle = np.abs(ee_orientation)))
        
    #only 11 and 12 configurations
    def test_inverse_t11_t12(self):
        theta_for_test = np.array([
            [0,        0,       np.pi/2,  0,       0], #t11 t12   | joint| joint
            [0,        0,       np.pi/4,  np.pi/4, 0], #t11 t12   | link | link
            [0,        0,       0,        0,       0], #t11 t12   | link | link
        ])
        for i in range(len(theta_for_test)):
            ee_orientation = theta_for_test[i][1] + theta_for_test[i][2] + theta_for_test[i][3]
            coordinates = self.youbot.get_end_effector_coors(theta_for_test[i])

            for conf_3 in range(1,3):
                assert self.youbot.get_end_effector_coors(
                    self.youbot.inverse_get_theta_array(
                    coordinates, 
                    conf_t_1 = 1, 
                    conf_t_3 = conf_3, 
                    orientation_angle = np.abs(ee_orientation))).all() == coordinates.all()
                
            for conf_3 in range(1, 3):
                with pytest.raises(Exception):
                    self.youbot.get_end_effector_coors(
                    self.youbot.inverse_get_theta_array(
                    coordinates, 
                    conf_t_1 = 2, 
                    conf_t_3 = conf_3, 
                    orientation_angle = np.abs(ee_orientation)))
    
    #only 21 and 22 configurations
    def test_inverse_t21_t22(self):
        theta_for_test = np.array([
            [-np.pi/2,        0,       0,       -np.pi/4, 0], #link  link  t12 t22
        ])
        for i in range(len(theta_for_test)):
            ee_orientation = theta_for_test[i][1] + theta_for_test[i][2] + theta_for_test[i][3]
            coordinates = self.youbot.get_end_effector_coors(theta_for_test[i])

            for conf_3 in range(1, 3):
                with pytest.raises(Exception):
                    self.youbot.get_end_effector_coors(
                    self.youbot.inverse_get_theta_array(
                    coordinates, 
                    conf_t_1 = 1, 
                    conf_t_3 = conf_3, 
                    orientation_angle = np.abs(ee_orientation)))
                    
            for conf_3 in range(1,3):
                assert self.youbot.get_end_effector_coors(
                    self.youbot.inverse_get_theta_array(
                    coordinates, 
                    conf_t_1 = 2, 
                    conf_t_3 = conf_3, 
                    orientation_angle = np.abs(ee_orientation))).all() == coordinates.all()
    
    #all configurations
    def test_inverse_t11_t12_t21_t22(self):
        theta_for_test = np.array([
            [-np.pi/2, -np.pi/6, np.pi/4,  np.pi/4, 0]
        ])
        for i in range(len(theta_for_test)):
            ee_orientation = theta_for_test[i][1] + theta_for_test[i][2] + theta_for_test[i][3]
            coordinates = self.youbot.get_end_effector_coors(theta_for_test[i])
        
            for conf_1 in range(1,3):
                for conf_3 in range(1,3):
                    assert self.youbot.get_end_effector_coors(
                        self.youbot.inverse_get_theta_array(
                        coordinates, 
                        conf_t_1 = conf_1, 
                        conf_t_3 = conf_3, 
                        orientation_angle = np.abs(ee_orientation))).all() == coordinates.all()      
    
    #none configurations
    def test_inverse_nt(self):
        theta_for_test = np.array([
            [0,        np.pi/3, np.pi/4,  np.pi/2, 0], #t11 joint | link | link
        ])
        for i in range(len(theta_for_test)):
            ee_orientation = theta_for_test[i][1] + theta_for_test[i][2] + theta_for_test[i][3]
            coordinates = self.youbot.get_end_effector_coors(theta_for_test[i])
            coordinates[0] +=0.1
            for conf_1 in range(1,3):
                for conf_3 in range(1,3):
                     with pytest.raises(Exception):
                        self.youbot.get_end_effector_coors(
                            self.youbot.inverse_get_theta_array(
                            coordinates, 
                            conf_t_1 = conf_1, 
                            conf_t_3 = conf_3, 
                            orientation_angle = np.abs(ee_orientation)))    