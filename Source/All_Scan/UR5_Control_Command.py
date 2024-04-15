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
robot.set_payload(0.8, (0, 0, 0.1))
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
Camera_Origin = np.array([-0.475555, -0.502177, -0.009607, -1.370543, -2.813686, -0.000120])
Robot_Home = np.array([-0.225319, -0.085779, 0.062316, -2.300640, -2.114415, -0.004161])
Get_Sample_Pos_L = np.array([-0.353782, -0.569349, 0.392509, -0.112911, -2.178284, 2.196601])
Get_Sample_Pos_J = np.array([0.084205, -1.538248, 2.126619, -3.853473, -0.175900, 1.714336])

# Glove Final Pos
gp = np.array([0.7982321381568909, -1.0429824034320276, 2.1144537925720215, -2.480995003377096, 1.5624452829360962, 0.7140231132507324])
##################################################################################################
# ur5m.movej(robot, list(Get_Sample_Pos_J), tcp, a, v)

p = Get_Sample_Pos_J + np.array([0, 0, 0, 0, 0, pi/2])
for i in range(37):
    if i < 36:
        ur5m.movej(robot, list(p), tcp, a, v)
    else:
        # ur5m.movej(robot, list(gp), tcp, a, v)    
        pass   
    while 1:
        try:
            ur5m.writeCameraLog('1')
            time.sleep(3)
            break
        except:
            print("Error! _ ", i)
            pass
    p += np.array([0, 0, 0, 0, 0, -pi/18])
ur5m.writeCameraLog('Done')

### END Glove Processing
# ur5m.movej(robot, list(Get_Sample_Pos_J), tcp, a, v)
############################################################################################

robot.stopl()
robot.stopj()
robot.stop()

print(f"Current Position: {robot.getl()}")
print(f"Current Position: {robot.getj()}")
print("Complete!")
robot.close()