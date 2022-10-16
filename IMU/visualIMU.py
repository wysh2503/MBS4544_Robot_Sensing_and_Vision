from vpython import *
from time import *
import numpy as np
import math
import serial
arduinoData = serial.Serial('com4',115200) #change com port
sleep(1)

scene.range=5
scene.background=color.gray(.2)
scene.forward=vector(-1,-1,-1)

scene.width=1200
scene.height=1080

xarrow = arrow(length=2, shaftwidth=.15, color=color.red, axis=vector(1,0,0))
yarrow = arrow(length=2, shaftwidth=.15, color=color.green, axis=vector(0,1,0))
zarrow = arrow(length=2, shaftwidth=.15, color=color.blue, axis=vector(0,0,1))

label(pos=vec(0,-2.5,0), color=color.blue, text='<b>IVE(Lee Wai Lee)\nDepartment of Engineering<b>', height=50)

frontArrow = arrow(length=2,shaftwidth=.1, color=color.purple, axis=vector(1,0,0))
upArrow = arrow(length=2,shaftwidth=.1, color=color.magenta, axis=vector(0,1,0))
sideArrow = arrow(length=2,shaftwidth=.1, color=color.orange, axis=vector(0,0,1))

breadBoard = box(length=2.5, width=5, height=.2, opacity=.8, pos=vector(0,0,0), color=color.yellow)
IMUsensor = box(length=.7, width=1, height=.1, pos=vector(-.2,0.1+.05,0), color=color.blue)
Arduino = box(length=1.2, width=1.5, height=.1, pos=vector(0,0.1+0.05,1.5), color=color.green)

myObj = compound([breadBoard, IMUsensor, Arduino])

while (True):
    try:
        while arduinoData.inWaiting() == 0:
            pass
        dataPacket = arduinoData.readline()
        dataPacket = str(dataPacket, 'utf-8')
        splitPacket = dataPacket.split(",")
        pitch = float(splitPacket[7])*2*np.pi/360. # change recevived data starting value [i]
        roll = float(splitPacket[8])*2*np.pi/360.
        yaw = -float(splitPacket[9])*2*np.pi/360.
        print("Roll=", roll / (2*np.pi/360.), " Pitch=", pitch / (2*np.pi/360.),
              "Yaw=", yaw / (2*np.pi/360.))

        rate(50)
        k = vector(cos(yaw)*cos(pitch), sin(pitch),sin(yaw)*cos(pitch))
        y = vector(0, 1, 0)
        s = cross(k, y)
        v = cross(s, k)
        vrot = v*cos(roll)+cross(k, v)*sin(roll)

        frontArrow.axis = k
        sideArrow.axis = cross(k, vrot)
        upArrow.axis = vrot
        myObj.axis = k
        myObj.up = vrot
        sideArrow.length = 3
        frontArrow.length = 4
        upArrow.length = 3
    except:
        pass
