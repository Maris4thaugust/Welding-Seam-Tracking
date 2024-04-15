import numpy as np
import time 
import socket

robot_ip = ("192.168.1.45", 30002)

def wait_move(robot):
    time.sleep(0.5)
    while 1:
        time.sleep(0.5)
        if not robot.is_program_running():
            print("Finished!")
            break

def movel(robot, p, tcp, a, v):
    # p = [x, y, z, Rx, Ry, Rz]
    movel_command = f"""
    def movel_ur5():
        set_tcp(p{tcp})
        movel(p{p}, {a}, {v})
    end
    """
    s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    s.connect(("192.168.1.45", 30002))
    s.send(movel_command.encode('utf-8'))
    time.sleep(0.1)
    wait_move(robot)
    s.close()

def movej(robot, q, tcp, a, v):
    # q = [q1, q2, q3, q4, q5, q6]
    movej_command = f"""
    def movej_ur5():
        set_tcp(p{tcp})
        movej({q}, {a}, {v})
    end
    """
    s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    s.connect(("192.168.1.45", 30002))
    s.send(movej_command.encode('utf-8'))
    time.sleep(0.1)
    wait_move(robot)
    s.close()

def gripper_open(robot):
    with open("gripper_open.script", "r", encoding='utf-8') as f:
        gripper_open = f.read()
    s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    s.connect(("192.168.1.45", 30002))
    s.send(gripper_open.encode())
    time.sleep(0.1)
    wait_move(robot)
    s.close()

def gripper_close(robot):
    with open("gripper_close.script", "r", encoding='utf-8') as f:
        gripper_close = f.read()
    s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    s.connect(("192.168.1.45", 30002))
    s.send(gripper_close.encode())
    wait_move(robot)
    time.sleep(0.5)
    s.close()

def writeCameraLog(txt):
    i = 0 # Check signal
    while (1):
        try:
            f = open("UR5_Camera_Log.txt", "wt")
            fr = open("UR5_Camera_Log.txt", "rt")
            break
        except:
            pass
    cnt = fr.read()
    while (1):
        i += 1
        if cnt == "100" or cnt == "":
            f.write(txt)
            break
        else:
            time.sleep(1)
            pass
        if i > 10:
            print("Error Step!")
            break
    f.close()
    fr.close()