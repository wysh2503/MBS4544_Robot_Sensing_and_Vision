from vpython import *
from time import *
import numpy as np
import math
import serial
arduinoData = serial.Serial('com6',115200)
sleep(1)

scene.range=5
scene.background=color.yellow
scene.forward=vector(-1,-1,-1)

scene.width=1200
scene.height=1080

xarrow = arrow(length=2, shaftwidth=.1, color=color.red,axis=vector(1,0,0))
yarrow = arrow(length=2, shaftwidth=.1, color=color.green,axis=vector(0,1,0))
zarrow = arrow(length=2, shaftwidth=.1, color=color.blue,axis=vector(0,0,1))

frontArrow=arrow(length=4,shaftwidth=.1,color=color.purple,axis=vector(1,0,0))
upArrow=arrow(length=1,shaftwidth=.1,color=color.magenta,axis=vector(0,1,0))
sideArrow=arrow(length=1,shaftwidth=.1,color=color.orange,axis=vector(0,0,1))

breadBoard = box(length=6,width=2,height=.2,opacity=.8,pos=vector(0,0,0,))
IMUsensor = box(length=1,width=.75,height=.1, pos=vector(-.5,.1+.05,0),color=color.blue)
Arduino = box(length=1.5,width=1.2,height=.1,pos=vector(-2.2,.1+.05,0),color=color.green)

myObj = compound([breadBoard, IMUsensor, Arduino])

while (True):
    try:
        while (arduinoData.inWaiting()==0):
            pass
        dataPacket=arduinoData.readline()
        dataPacket=str(dataPacket,'utf-8')
        splitPacket=dataPacket.split(",")
        pitch = float(splitPacket[15])*2*np.pi/360.
        roll = float(splitPacket[16])*2*np.pi/360.
        yaw = float(splitPacket[17])*2*np.pi/360.
        print("Roll=", roll / (2*np.pi/360.), " Pitch=", pitch / (2*np.pi/360.),
              "Yaw=",yaw / (2*np.pi/360.))

        rate(50)
        k=vector(cos(yaw)*cos(pitch), sin(pitch),sin(yaw)*cos(pitch))
        y=vector(0,1,0)
        s=cross(k,y)
        v=cross(s,k)
        vrot=v*cos(roll)+cross(k,v)*sin(roll)

        frontArrow.axis=k
        sideArrow.axis=cross(k,vrot)
        upArrow.axis=vrot
        myObj.axis=k
        myObj.up=vrot
        sideArrow.length=1
        frontArrow.length=4
        upArrow.length=1
    except:
        pass