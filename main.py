import youbot
import matplotlib.pyplot as plt
import numpy as np
np.set_printoptions(precision=4, suppress=True)

youbot = youbot.Youbot()

# x = 0.3
# y = 0.2
# z = 0.2
# coordinates = np.array([x,y,z])

# print("coor = " ,coordinates)



theta_for_test = np.array([
        [0,        np.pi/3, np.pi/4,  np.pi/2, 0],
        [0,        np.pi/3, np.pi/4,  np.pi/4, 0],
        [0,        0,       np.pi/2,  0,       0],
        [0,        0,       np.pi/4,  np.pi/4, 0],
        [0,        0,       0,        0,       0],
        [-np.pi/2,        0,       0,       -np.pi/4, 0],
        [-np.pi/2, 0,       0,        0,       0],
        [-np.pi/2, 0,       np.pi/4,  np.pi/4, 0],
        [-np.pi/2, 0,       np.pi/2,  0,       0],
        [-np.pi/2, np.pi/3, np.pi/4,  np.pi/4, 0],
        [-np.pi/2, np.pi/3, np.pi/4,  np.pi/2, 0],
        [-np.pi/2, -np.pi/6, np.pi/4,  np.pi/4, 0],
    ])

for i in range(11):
    for conf_t_1 in range(2):
        for conf_t_2 in range(2):
            print("-----",conf_t_1 + 1, conf_t_2 + 1, " test N", i, "WORKING... ----")
            
            # set some angles
            theta_array = theta_for_test[i]
            
            # goal view
            youbot.view_model.plot(theta_array, block=True)
            
            # forward get goal coordinates
            coordinates_f = youbot.get_end_effector_coors(theta_array)
            
            ee_y_orientation = theta_array[1]+theta_array[2]+theta_array[3]
            try:
                # inverse get theta array by goal coor + orientation
                theta_array_m = youbot.inverse_get_theta_array(coordinates_f, conf_t_1 + 1, conf_t_2 + 1, np.abs(ee_y_orientation) )
                
                # coors
                coordinates_2 = youbot.get_end_effector_coors(theta_array_m)
                
                print("-----",conf_t_1 + 1, conf_t_2 + 1, " test N", i, "DONE ---------")
                print("theta_array = ", theta_array)
                print("coordinates_f = ", coordinates_f)
                print("theta_array_f = ", theta_array_m)
                print("coordinates_2 = ", coordinates_2)
                print("---------------------------------------------")
                
                # view
                youbot.view_model.plot(theta_array_m, block=True)
            except Exception as err:
                print(err.args)



plt.show()