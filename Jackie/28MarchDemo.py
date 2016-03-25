# -*- coding: utf-8 -*-
"""
Created on Sat Feb 20 14:36:33 2016

@author: clyman
"""

import serial
import numpy
import time
import matplotlib.pyplot as plt


def draw_robot(pos, rad, theta): #function to draw circle representing robot
    the = numpy.linspace(0, 2*numpy.pi, 100, endpoint=True)
    xunit = rad * numpy.cos(the) + pos[0]
    yunit = rad * numpy.sin(the) + pos[1]
    
    center = pos
    front = [0, 0]
    front[0] = center[0] + rad*numpy.cos(theta)
    front[1] = center[1] + rad*numpy.sin(theta)
    
    plt.plot([center[0], front[0]], [center[1], front[1]], 'b-', xunit, yunit, 'b-')
    plt.draw()

plt.ion()
robot_position = [12, -15]
robot_radius = 3
robot_theta = 0
new_pos = [0, 0]
dist_traveled = 0

straight_gain = 1./791.
straing_std = 30

plt.figure(1)
#plt.subplot(211)
plt.hold(True)
plt.axis([-10, 110, -70, 10])
plt.plot([0, 102], [0, 0], 'b-') #Create box representing boundaries
plt.plot([102,102], [0, -62], 'b-')
plt.plot([102, 0], [-62, -62], 'b-')
plt.plot([0, 0], [-62, 0], 'b-')

#plt.figure(1)
#plt.subplot(212)
#plt.hold(True)
#plt.axis([0, 32, 0, 5.1])
#plt.xlabel("Time (s)")
#plt.ylabel("Voltage (V)")
#plt.title("Voltage vs. Time")


def read_robot(): 
    encoder = numpy.zeros(500)
    turn = numpy.zeros(500)
    seconds = numpy.zeros(500)
    checkDriveCharacter = '$1'
    checkTurnCharacter = '$3'
    breakCheck = '$0'
    counter = 0
    while True:
        charToRead = 24
        check = robot.read(2)
        if check == checkDriveCharacter or check == checkTurnCharacter:
            b = robot.inWaiting()
            if b >= charToRead:
                msg = robot.readline()
                encoder[counter] = int(msg[0:7])
                turn[counter] = int(msg[7:12])
                seconds[counter] = float(msg[12:20]) / 1000.
                counter += 1
                print msg
            if check == checkDriveCharacter:
                new_pos[0] = robot_position[0] + encoder * numpy.cos(robot_theta)
                new_pos[1] = robot_position[1] + encoder * numpy.sin(robot_theta)
                plt.figure(1)
                plt.plot([robot_position[0], new_pos[0]], [robot_position[1], new_pos[1]])
                robot_position[0] = new_pos[0]
                robot_position[1] = new_pos[1]
                dist_traveled = dist_traveled + encoder
            elif check == checkTurnCharacter:
                turn = turn * (numpy.pi / 180) * (-1)
                robot_theta = turn + robot_theta
                dist_traveled = -15
                    
            
        elif check == breakCheck:
            break
    

def drive_robot(drive,brake):
    robot.flushInput()
    robot.flushOutput()
    time.sleep(0.2)
    robot.write(drive)
    time.sleep(0.3)
    read_robot()
    brake_robot(brake)    

def brake_robot(brake):
    robot.flushInput()
    robot.flushOutput()
    time.sleep(0.2)
    robot.write(brake)
    time.sleep(0.2)


   
'''.....................................................................................'''



'''!!!!!CHANGE PARAMETERS BASED ON WHAT IS PLUGGED IN!!!!!'''

drive = '$11000' #drive 40 inches
brake = '$00000'



PORT = '/dev/tty.usbserial-DA011NKM' 
#PORT = '/dev/tty.usbmodem1411'
BaudRate = 38400
robot = serial.Serial(PORT, BaudRate, timeout = 3)
time.sleep(0.5)
if robot.isOpen():
    print 'Robot opened\n'

robot.write(brake)
time.sleep(1)    
robot.flushInput()
robot.flushOutput()
    
drive_robot(drive, brake)
time.sleep(3)

robot.close()

if robot.isOpen() == False:
    print '\nRobot closed'