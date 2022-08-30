import youbot
import matplotlib.pyplot as plt
import numpy as np
np.set_printoptions(precision=4, suppress=True)

youbot = youbot.Youbot()

x = 1
y = 1
z = 1
coordinates = np.array([x,y,z])

print("coor = " ,coordinates)

try:

    theta_array = youbot.inverse_get_theta_array(coordinates, conf_t_1 = 1, conf_t_3 = 1, orientation_angle = -np.pi/4)

    print("ozk thetas =", theta_array)


    print("coor test ozk by pzk =", youbot.get_end_effector_coors(theta_array))
    youbot.view_model.plot(theta_array, block=True)
    
    
except Exception as err:
    print(err.args)

plt.show()