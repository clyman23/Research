import serial
import numpy as np
import time
import matplotlib.pyplot as plt


def openPlot():
    plt.ion()
    robot_position = [12, -15] #starting position
    robot_radius = 3 #size of robot representation
    robot_theta = 0 #angle robot is pointed at
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
    return robot_position, robot_radius, robot_theta


def read_robot(): 
    encoder = np.zeros(500)
    seconds = np.zeros(500)
    current = np.zeros(500)
    charge = np.zeros(500)
    counter = 0
    total_charge = 0
    while True:
        charToRead = 24
        checkCharacter = '$1'
        breakCheck = '$0'
        check = robot.read(2)
        if check == checkCharacter:
            b = robot.inWaiting()
            if b >= charToRead:
                msg = robot.readline()
                encoder[counter] = int(msg[0:8])
                seconds[counter] = float(msg[9:16]) / 1000.
                current[counter] = float(msg[17:]) / 1000.
                charge[counter] = current[counter] * seconds[counter]
                total_charge += charge[counter]
                counter += 1
                print msg
        elif check == breakCheck:
            break
    
    

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

drive = '$13300' 
brake = '$00000'
turn90 = '$00013'

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
robotInitials = np.array(openPlot()) #[robot position, robot radius, robot theta]


drive_robot(drive, brake)
time.sleep(3)

robot.close()

if robot.isOpen() == False:
    print '\nRobot closed'