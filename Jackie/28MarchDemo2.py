# -*- coding: utf-8 -*-
"""
Created on Fri Mar 25 15:50:30 2016

@author: clyman
"""

import serial
import time
import matplotlib.pyplot as plt
import numpy as np

def draw_robot(pos, rad, theta): #function to draw circle representing robot
    the = np.linspace(0, 2*np.pi, 100, endpoint=True)
    xunit = rad * np.cos(the) + pos[0]
    yunit = rad * np.sin(the) + pos[1]
    
    center = pos
    front = [0, 0]
    front[0] = center[0] + rad*np.cos(theta)
    front[1] = center[1] + rad*np.sin(theta)
    
    plt.plot([center[0], front[0]], [center[1], front[1]], 'b-', xunit, yunit, 'b-')
    plt.draw()

plt.ion() #interactive mode http://matplotlib.org/faq/usage_faq.html#what-is-interactive-mode 
robot_position = [12, -15] #starting position
robot_radius = 3 #size of robot representation
robot_theta = 0 #angle robot is pointed at

straight_gain = 1.0/791.0 # 791 encoder counts per inch
straight_std = 30 #unsure what this is for

plt.figure(1)
plt.subplot(211)
plt.hold(True)
plt.axis([-10, 110, -70, 10])
plt.plot([0, 102], [0, 0], 'b-') #Create box representing boundaries
plt.plot([102,102], [0, -62], 'b-')
plt.plot([102, 0], [-62, -62], 'b-')
plt.plot([0, 0], [-62, 0], 'b-')

plt.figure(1)
plt.subplot(212)
plt.hold(True)
plt.axis([0, 32, 0, 5.1])
plt.xlabel("Time (s)")
plt.ylabel("Voltage (V)")
plt.title("Voltage vs. Time")
#plt.text(25, 4, "Red = Solar\nBlack = Thermo")

PORT = '/dev/tty.usbserial-DA011NKM' 
robot = serial.Serial(PORT, 38400, timeout = 1) 

if robot.isOpen() == True:
    print "Port opened"

robot.flushInput()
robot.flushOutput()

new_pos = [0, 0] #Initializing lists to be used later
all_index = []

old_seconds = 0
old_solar = 0
old_thermo = 0
dist_traveled = 12 #distance traveled by robot


robot.flushInput()
robot.flushOutput()
drive = '$11000' #drive 10 inches
time.sleep(0.2)
robot.write(drive)
time.sleep(3)
turn = '$00012'
robot.write(turn)
time.sleep(3)
robot.write(drive)

while len(all_index) < 50: 
    time.sleep(.1)
    b = robot.inWaiting()
    
    while b < 24: #Wait for 24 bytes to be sent
        b = robot.inWaiting()
    
    if b > 24: #Ensure that 24 bytes are read (the length of one data thread)
        b = 24

    msg = robot.readline()
    index = float(msg[1])
    dist = float(msg[2:9])
    turnangleave = float(msg[10:14])
    seconds = float(msg[15:22])
    solarVoltage = 0
    
    all_index.append(index)
    
    if (index==1): #Plot the robot's path
        dist = dist * straight_gain
        new_pos[0] = robot_position[0] + dist*np.cos(robot_theta)
        new_pos[1] = robot_position[1] + dist*np.sin(robot_theta)
        plt.figure(1)
        plt.subplot(211)
        plt.plot([robot_position[0], new_pos[0]], [robot_position[1], new_pos[1]], 'g-')
        robot_position[0] = new_pos[0]
        robot_position[1] = new_pos[1]
        dist_traveled = dist_traveled + dist
        #h = draw_robot(robot_position, robot_radius, robot_theta)
        
    elif index==3: #Robot turning right
        turnangleave = turnangleave * (np.pi / 180) * (-1)
        robot_theta = turnangleave + robot_theta
        dist_traveled = -15
        #h = draw_robot(robot_position, robot_radius, robot_theta)
     
        
    if solarVoltage>=4.70: #Plot sources of solar power
        plt.figure(1)
        plt.subplot(211)
        plt.plot(robot_position[0], robot_position[1], 'y*')
        
    plt.figure(1)
    plt.subplot(212)
    seconds = seconds / 1000
    plt.plot([old_seconds, seconds] , [old_solar, solarVoltage], 'r-')
#    plt.plot([old_seconds, seconds], [old_thermo, thermoVoltage], 'k-')
    old_seconds = seconds
    old_solar = solarVoltage
#    old_thermo = thermoVoltage
    plt.draw()
    
    plt.figure(1)
    plt.draw()
    
    
plt.figure(1)
plt.subplot(211)
h = draw_robot(robot_position, robot_radius, robot_theta)



robot.close()