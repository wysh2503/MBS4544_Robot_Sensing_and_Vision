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

scene.width=1600
scene.height=1080

# Add text/label to scene
# label(pos=vec(0,-2.5,0), color=color.blue, text='<b>IVE(Lee Wai Lee)\nDepartment of Engineering<b>', height=50)
text = text(text='IVE(Lee Wai Lee)\nDepartment of Engineering', axis=vector(1,0,0), pos=vector(0,-2.5,0),
    align='center', height=0.6, depth=0.3, color=color.cyan)

# Global coordinate
xarrow = arrow(length=2, shaftwidth=.15, color=color.red, axis=vector(1,0,0))
yarrow = arrow(length=2, shaftwidth=.15, color=color.green, axis=vector(0,1,0))
zarrow = arrow(length=2, shaftwidth=.15, color=color.blue, axis=vector(0,0,1))

# Object coordinate
frontArrow = arrow(length=4,shaftwidth=.1, color=color.purple, axis=vector(1,0,0))
upArrow = arrow(length=3,shaftwidth=.1, color=color.magenta, axis=vector(0,1,0))
sideArrow = arrow(length=3,shaftwidth=.1, color=color.orange, axis=vector(0,0,1))

breadBoard = box(length=3, width=5, height=0.2, opacity=.8, pos=vector(0,0,0), color=color.yellow)
IMUsensor = box(length=.7, width=1, height=0.1, pos=vector(0,0.1+0.05,0), color=color.blue)
Arduino = box(length=2, width=1.2, height=0.1, pos=vector(0,0.1+0.05,1.5), color=color.green)

myObj = compound([breadBoard, IMUsensor, Arduino])

while (True):
    try:
        while arduinoData.inWaiting() == 0:
            pass
        dataPacket = arduinoData.readline()
        dataPacket = str(dataPacket, 'utf-8')
        splitPacket = dataPacket.split(",")
        pitch_rad = float(splitPacket[7])*2*np.pi/360  # change received data starting value [i]
        roll_rad = float(splitPacket[8])*2*np.pi/360
        yaw_rad = float(splitPacket[9]) * 2*np.pi/360 + np.pi
        
        pitch_deg = pitch_rad / (2*np.pi/360)
        roll_deg = roll_rad / (2*np.pi/360)
        yaw_deg = yaw_rad / (2*np.pi/360)
        print("Roll=", roll_deg, " Pitch=", pitch_deg, "Yaw=",yaw_deg)

        rate(50)
        k = vector(cos(yaw_rad)*cos(pitch_rad), sin(pitch_rad),sin(yaw_rad)*cos(pitch_rad))
        y = vector(0, 1, 0)
        s = cross(k, y)
        v = cross(s, k)
        vrot = v*cos(roll_rad)+cross(k, v)*sin(roll_rad)

        frontArrow.axis = k
        sideArrow.axis = cross(k, vrot)
        upArrow.axis = vrot
        myObj.axis = k
        myObj.up = vrot
        sideArrow.length = 3
        frontArrow.length = 4
        upArrow.length = 3
        
        label(pos=vec(0, 4, 0), color=color.blue,
              text='<b>Roll:<b> {:7.2f} <b> Pitch:<b> {:7.2f} <b> Yaw:<b> {:7.2f} <b>'.format(
                  roll_deg,pitch_deg,yaw_deg), height=50, font='sans')
    except:
        pass
