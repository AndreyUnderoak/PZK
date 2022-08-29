import pytest
import youbot
import numpy as np

class Test_class:
    youbot = youbot.Youbot()

    def test_inverse_1(self):
        x = 0.15
        y = 0.15
        z = 0.4
        coordinates = np.array([x,y,z])

        assert self.youbot.get_end_effector_coors(
            self.youbot.inverse_get_theta_array(
            coordinates, 
            conf_t_1 = 1, 
            conf_t_3 = 1, 
            orientation_angle = -np.pi/4)) == coordinates