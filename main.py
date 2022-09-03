import youbot
import matplotlib.pyplot as plt
import numpy as np
np.set_printoptions(precision=4, suppress=True)

youbot = youbot.Youbot()

x = 0.3
y = 0.2
z = 0.2
coordinates = np.array([x,y,z])

print("coor = " ,coordinates)

try:

    # theta_array = youbot.inverse_get_theta_array(coordinates, conf_t_1 = 1, conf_t_3 = 1, orientation_angle = -np.pi/4)

    # print("ozk thetas =", theta_array)


    # print("coor test ozk by pzk =", youbot.get_end_effector_coors(theta_array))
    
    
    theta_array = np.array([0,np.pi/8,np.pi/8, np.pi/4, 0])
    
    
    youbot.view_model.plot(theta_array, block=True)
    
    coordinates_f = youbot.get_end_effector_coors(theta_array)
    
    
    theta_array_m = youbot.inverse_get_theta_array(coordinates_f, 1, 1, np.pi/2,0)
    
    
    youbot.view_model.plot(theta_array_m, block=True)
    
    coordinates_2 = youbot.get_end_effector_coors(theta_array_m)
    print("theta_array = ", theta_array)
    print("coordinates_f = ", coordinates_f)
    print("theta_array_f = ", theta_array_m)
    print("coordinates_2 = ", coordinates_2)
    
except Exception as err:
    print(err.args)

plt.show()