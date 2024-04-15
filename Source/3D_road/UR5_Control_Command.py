from urx.robot import Robot
import numpy as np
import time 
import os

import UR5_necessaryMethod as ur5m

##################################################################################################

cwd = os.getcwd()
os.system(f"cd {cwd}")

##################################################################################################
# Ket noi va khoi tao Robot
robot = Robot("192.168.1.45")
robot.set_tcp((0, 0, 0.275, 0, 0, 0))
robot.set_payload(0.3, (0, 0, 0.1))
time.sleep(1)

##################################################################################################
# Constant parameters
pi = np.pi
a = 0.1 # acceleration
v = 0.1 # velocity
r = 0 # radius (use 0 for linear motion)
tcp = np.array([-0.002474130126820129, -0.0018488067166400612, 0.275, 0.0, 0.0, 0.0])

##################################################################################################
# Special points
Camera_Origin = np.array([0.64, -1.11, 1.21, -1.70, -1.56, 0.69])
Robot_Home = np.array([0.02, -2.00, 1.94, -1.55, -1.60, 0.08])
Jto_Cap = np.array([-0.176, -1.9306, 2.037, -0.256, -0.1209, -1.381])

##################################################################################################
# step1: 
ur5m.movej(robot, list(Jto_Cap), tcp, a, v)

# step2: 
p = np.array(robot.getj())
for i in range(5):
    ur5m.movej(robot, list(p), tcp, a, v)
    p += np.array([0, 0, 0, 0, 0, -10*pi/180])  
    while 1:
        try:
            ur5m.writeCameraLog('1')
            time.sleep(3)
            break
        except:
            pass
ur5m.movej(robot, list(Jto_Cap), tcp, a, v)
p = np.array(robot.getj())
for i in range(5):
    ur5m.movej(robot, list(p), tcp, a, v)
    p += np.array([0, 0, 0, 0, 0, 10*pi/180])  
    while 1:
        try:
            ur5m.writeCameraLog('1')
            time.sleep(3)
            break
        except:
            pass
ur5m.movej(robot, list(Jto_Cap), tcp, a, v)

### END Shoe Processing
# ur5m.movej(robot, list(Get_Sample_Pos_J), tcp, a, v)
############################################################################################

robot.stopl()
robot.stopj()
robot.stop()

print(f"Current Position: {robot.getl()}")
print(f"Current Position: {robot.getj()}")
print("Complete!")
robot.close()