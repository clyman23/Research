# -*- coding: utf-8 -*-
"""
Created on Sat Feb 20 14:36:33 2016

@author: clyman
"""

import serial
import numpy
import time
import csv


def read_robot(): 
    time.sleep(0.5)    
    encoder = numpy.zeros(500)
    seconds = numpy.zeros(500)
    current = numpy.zeros(500)
    charge = numpy.zeros(500)
    counter = 0
    encoderCheck = 2000
    total_charge = 0
    
    while True:
#        charToRead = 21
        checkCharacter = '$1'
        breakCheck = '$0'
#        print robot.inWaiting()
        check = robot.read(2)
#        print check
        if check == checkCharacter:
    #        while robot.inWaiting():
#            b = robot.inWaiting()
            #time.sleep(0.05) #try uncommenting for USB communication
            #print b
#            if b >= charToRead:
    #                if b > charToRead:
    #                    b = charToRead
            msg = robot.readline()
            if int(msg[0:7]) > encoderCheck:
                encoder[counter] = int(msg[0:7])
                seconds[counter] = float(msg[8:13]) / 1000.
                current[counter] = float(msg[14:]) / 1000.
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
    print "driving"
    robot.flushInput()
    robot.flushOutput()
    time.sleep(0.3)
    robot.write(drive)
    time.sleep(0.3)
    charge = read_robot()
    brake_robot(brake)
    return charge
    

def brake_robot(brake):
    print "braking"
    #robot.flushInput()
    #robot.flushOutput()
    time.sleep(0.2)
    robot.write(brake)
    time.sleep(0.2)
    #robot.flushInput()
    #robot.flushOutput()
    


   
'''.....................................................................................'''



'''!!!!!CHANGE PARAMETERS BASED ON WHAT IS PLUGGED IN!!!!!'''

dist = 33
drive = '$1'+str(dist)+'00' #drive dist inches
brake = '$00000'


#PORT = '/dev/tty.usbserial-DA011NKM' 
PORT = '/dev/tty.usbmodem1411'
#PORT = 'COM4'
BaudRate = 38400
robot = serial.Serial(PORT, BaudRate, timeout = 2)
time.sleep(0.5)
if robot.isOpen():
    print 'Robot opened\n'

#brake_robot(brake)
time.sleep(1)    
#robot.flushInput()
#robot.flushOutput()

for i in range(0,5): # range(0,5), then range(5,10), (10,15) ...
    file_name = '{0:02d}_testApr21_{1:02d}.csv'.format(dist,i)
    data_file = open(file_name,'wb')
    time.sleep(0.1)
    writer = csv.writer(data_file)
    writer.writerow(('encoder', 'seconds', 'current (A)', 'charge (C)'))
    time.sleep(0.1)
    charge = drive_robot(drive, brake)
    print "The total charge for this test is " + str(charge)
    time.sleep(3)
    data_file.close()
    robot.flushInput()
    robot.flushOutput()
    
#drive_robot(drive,brake)

robot.close()

if robot.isOpen() == False:
    print '\nRobot closed'