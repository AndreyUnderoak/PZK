import youbot
import matplotlib.pyplot as plt
import numpy as np
np.set_printoptions(precision=4, suppress=True)

youbot = youbot.Youbot()

#TRAJECTORY
t0 = 0
t1 = 3
t = np.array([
        [t0**3,    t0**2, t0, 1],
        [3*t0**2,  2*t0,  1,  0],
        [t1**3,    t1**2, t1, 1],
        [3*t1**2,  2*t1,  1,  0],
    ])

print("t = ", t)

step = 0.05
time = np.arange(t0, t1 + step, step)

x = np.array([0.16, 0, 0.26, 0])
    
y = np.array([0, 0, 0, 0])
    
z = np.array([0.2, 0, 0.2, 0])
    
print("It = ", np.linalg.inv(t))
    
def a(p):
    return np.dot(np.linalg.inv(t), p)

ax = a(x)
ay = a(y)
az = a(z)

print('Test\n', t.dot(ax))


xyz_matrix = np.array([ax, ay, az])
print('xyz\n', xyz_matrix)
def f(xyz_matrix, t):
    f = np.zeros(3)
    for i in range(3):
        f[i] = (xyz_matrix[i][0] * t**3 + xyz_matrix[i][1] * t**2 + xyz_matrix[i][2] * t + xyz_matrix[i][3])
    return f


ee_y_orientation = np.pi/2 + np.pi/4
coordinates_plot = []
th = []

for tt in time:
    coordinates = f(xyz_matrix, tt)
    print("coor = " , coordinates)
    try:    
        theta_array = youbot.inverse_get_theta_array(coordinates, 1, 1, np.abs(ee_y_orientation))
        print("thetas = " , theta_array)
        th.append(theta_array)
    except Exception as err:
        print(err.args)
        
    # coordinates_plot = np.append(coordinates_plot, [coordinates])
    coordinates_plot.append(coordinates)

th = np.array(th)
coordinates_plot = np.array(coordinates_plot)


print(time)
fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')
ax.plot(coordinates_plot[:, 0], coordinates_plot[:, 1], coordinates_plot[:, 2], label='1')
plt.show()

plt.plot(time, coordinates_plot[:, 0])
plt.show()

plt.plot(time, coordinates_plot[:, 1])
plt.show()

plt.plot(time, coordinates_plot[:, 2])
plt.show()

plt.plot(time, th[:, 2])
plt.show()

# goal view
# youbot.view_model.plot(theta_array, block=True)

# forward get goal coordinates
# coordinates_f = youbot.get_end_effector_coors(theta_array)


# try:
#     # inverse get theta array by goal coor + orientation
#     theta_array_m = youbot.inverse_get_theta_array(coordinates, 1, 1, np.abs(ee_y_orientation) )
    
#     # coors
#     coordinates_2 = youbot.get_end_effector_coors(theta_array_m)
    
#     print("-----",conf_t_1 + 1, conf_t_2 + 1, " test N", i, "DONE ---------")
#     print("theta_array = ", theta_array)
#     print("coordinates_f = ", coordinates_f)
#     print("theta_array_f = ", theta_array_m)
#     print("coordinates_2 = ", coordinates_2)
#     print("---------------------------------------------")
    
#     # view
#     youbot.view_model.plot(theta_array_m, block=True)
# except Exception as err:
#     print(err.args)


