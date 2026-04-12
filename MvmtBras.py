from dynamixel_sdk import *  
import time 
from math import * 
import numpy as np
import matplotlib.pyplot as plt
import angle

DEVICENAME = 'COM7'       
BAUDRATE = 1000000
PROTOCOL_VERSION = 2.0
DXL_IDS = [1, 2, 3, 4, 5]
ADDR_TORQUE_ENABLE = 64
ADDR_GOAL_POSITION = 116
ADDR_PRESENT_POSITION = 132
TORQUE_ENABLE = 1
TORQUE_DISABLE = 0

ouvert = 1500
ferme = 2000

portHandler = PortHandler(DEVICENAME)
packetHandler = PacketHandler(PROTOCOL_VERSION)
if not portHandler.openPort():
    print("Erreur : impossible d’ouvrir le port.")
    quit()
if not portHandler.setBaudRate(BAUDRATE):
    print("Erreur : impossible de définir le débit.")
    quit()
for dxl_id in DXL_IDS:
    dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(
        portHandler, dxl_id, ADDR_TORQUE_ENABLE, TORQUE_ENABLE
    )



def go(a):
    for dxl_id, pos in zip(DXL_IDS, a):
        packetHandler.write4ByteTxRx(portHandler, dxl_id, ADDR_GOAL_POSITION, pos)



def servo(pos_in, ang):
    pos_fi = [0,0,0,0,0]
    for i in range(5):
        pos_fi[i] = pos_in[i]-int(ang[i])
    return pos_fi

def position(x, y):
    
    distance = sqrt(x**2 + y**2)

    if distance < 15:
        facteur = 15 / distance  
        x_corr = x * facteur
        y_corr = y * facteur
        return (False, (x_corr, y_corr))

    elif distance > 29:
        facteur = 29 / distance  
        x_corr = x * facteur
        y_corr = y * facteur
        return (False, (x_corr, y_corr))

    return (True, (x, y))

def relever():
    pos = [2000, 2000,1200, 2000, 1500] 
    go(pos)

def recup_pos():
    l = []
    for i in range(1,6):
        position_value, result, error = packetHandler.read4ByteTxRx(portHandler, i, ADDR_PRESENT_POSITION)
        l.append(position_value)
    return l 

def angle_fin(x,y,a):
    verif = position(x,y)
    if verif[0]==False:
        x,y=verif[1][0],verif[1][1]
    pos = [2000, 2800,1200, 2000, a] 
    angle_final = angle.ki(x,y)
    position_final = servo(pos,angle_final)
    return position_final


def attraper_objet(x,y):
    a = angle_fin(x,y,ouvert)
    go(a)
    time.sleep(4)
    a = angle_fin(x,y,ferme)
    go(a)
    time.sleep(4)
    relever()