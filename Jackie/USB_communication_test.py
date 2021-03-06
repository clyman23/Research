# -*- coding: utf-8 -*-
"""
Created on Fri Mar 11 14:06:28 2016

@author: clyman
"""

import serial
import numpy
import time
import csv


def read_robot(): 
    encoder = numpy.zeros(500)
    seconds = numpy.zeros(500)
    current = numpy.zeros(500)
    charge = numpy.zeros(500)
    counter = 0
    encoderCheck = 2000
    total_charge = 0
    while True:
        charToRead = 24
        checkCharacter = '$1'
        breakCheck = '$0'
        check = robot.read(2)
        if check == checkCharacter:
    #        while robot.inWaiting():
            b = robot.inWaiting()
    #            time.sleep(0.05)
            if b >= charToRead:
    #                if b > charToRead:
    #                    b = charToRead
                msg = robot.readline()
                if int(msg[0:8]) > encoderCheck:
                    encoder[counter] = int(msg[0:8])
                    seconds[counter] = float(msg[9:16]) / 1000.
                    current[counter] = float(msg[17:]) / 1000.
                    charge[counter] = current[counter] * seconds[counter]
                    total_charge += charge[counter]
                    counter += 1
                    print msg
     #           print "\n"
        elif check == breakCheck:
            break
        #elif check != checkCharacter and check != breakCheck:
         #   check = robot.read(2)
    write2csv(encoder, seconds, current, charge, counter)
    
    return total_charge
 

def write2csv(encoder, seconds, current, charge, counter):
    for i in range(counter):
        writer.writerow((encoder[i], seconds[i], current[i], charge[i]))
    writer.writerow([])
    

def drive_robot(drive,brake):
    robot.flushInput()
    robot.flushOutput()
    time.sleep(0.2)
    robot.write(drive)
    time.sleep(0.3)
    charge = read_robot()
    brake_robot(brake)
    return charge
    

def brake_robot(brake):
    robot.flushInput()
    robot.flushOutput()
    time.sleep(0.2)
    robot.write(brake)
    time.sleep(0.2)


   
'''.....................................................................................'''



'''!!!!!CHANGE PARAMETERS BASED ON WHAT IS PLUGGED IN!!!!!'''

drive = '$11300' #drive 40 inches
brake = '$00000'



#PORT = '/dev/tty.usbserial-DA011NKM' 
PORT = '/dev/tty.usbmodem1421'
BaudRate = 38400
robot = serial.Serial(PORT, BaudRate, timeout = 3)
time.sleep(0.5)
if robot.isOpen():
    print 'Robot opened\n'

robot.write(brake)
time.sleep(1)    
robot.flushInput()
robot.flushOutput()

'''
for i in range(0,5): # range(0,5), then range(5,10), (10,15) ...
    file_name = '13inchesMarch8Grass_0' + str(i) + '.csv'
    data_file = open(file_name,'wb')
    writer = csv.writer(data_file)
    writer.writerow(('encoder', 'seconds', 'current (A)', 'charge (C)'))
    
    charge = drive_robot(drive, brake)
    print "The total charge for this test is " + str(charge)
    time.sleep(3)
    data_file.close()
'''

for i in range(0,5):
    file_name = 'test_' + str(i) + '.csv'
    data_file = open(file_name, 'wb')
    writer = csv.writer(data_file)
    charge = drive_robot(drive,brake)
    print "Driven..." + str(i)
    time.sleep(3)
    data_file.close()

robot.close()

if robot.isOpen() == False:
    print '\nRobot closed'