import roboticstoolbox as rtb
import spatialmath as spm
import numpy as np
import matplotlib.pyplot as plt

#length of the YOUBOT's links 
l = [0.033, 0.147, 0.155, 0.135, 0.2175]

youbot = rtb.DHRobot(
[
    rtb.RevoluteDH(d=-l[1], a=l[0], alpha=np.pi/2, offset = 0),
    rtb.RevoluteDH(d=0, a=l[2], alpha=-np.pi, offset = -np.pi/2),
    rtb.RevoluteDH(d=0, a=l[3], alpha=np.pi, offset = 0),
    rtb.RevoluteDH(d=0, a=0, alpha=np.pi/2, offset = -np.pi/2),
    rtb.RevoluteDH(d=-l[4], a=0, alpha=0, offset = 0)
], base = spm.SE3.Rx(np.pi), name="YOUBOT")

print(youbot)

youbot.plot(youbot.q, block=True)


plt.show()