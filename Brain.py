import numpy as np
import bisect

from math import *
import logging
import time
import os
from gps import * #DISABLE FOR PC TESTING - (NO GPS)
from time import *
import threading
import gps3 #DISABLE FOR PC TESTING - (NO GPS)
import csv
import subprocess
import serial
import re
import glob
from bluetooth import *
from numpy import cross, eye, dot

base_dir = '/sys/bus/w1/devices/'
device_folder = '/sys/bus/w1/devices/28*'
device_file = '/sys/bus/w1/devices/28*/w1_slave'
server_sock = BluetoothSocket( RFCOMM )
server_sock.bind(("",PORT_ANY))
server_sock.listen(1)
port = server_sock.getsockname()[1]

uuid = "94f39d29-7d6d-437d-973b-fba39e49d4ee"

advertise_service( server_sock, "DART",
                   service_id = uuid,
                   service_classes = [ uuid, SERIAL_PORT_CLASS ],
                   profiles = [ SERIAL_PORT_PROFILE ], 
#                   protocols = [ OBEX_UUID ] 
                    )

filename = "test.csv"
filename2 = "test2.csv"

class State:

    def __init__(self, x=0.0, y=0.0, yaw=0.0, v=0.0):
        self.x = x
        self.y = y
        self.yaw = yaw
        self.v = v
        self.vlast = 0
        self.xlast = 0
        self.ylast = 0
        self.yawlast = 0
        self.dv = 0
        self.dyaw = 0
        self.dx = 0
        self.dy = 0
        self.DRv = 0
class Arduino(object):
    def __init__(self, meas):
        self.ArdSerial = None
        self.meas = meas
        self.datain = None
        self.data = None
        self.dataout = None
    def connectArduino(self):

        self.found = False
        counter = 0
        while not self.found:
            try:
                
                port = ("/dev/ttyACM%s" % counter)
                self.ArdSerial = serial.Serial(port, 9600)
                self.found = True
                
                
            except Exception as inst:
                print(inst)
                counter += 1
                if counter == 20:
                    print('Could not Connect Arduino')
                    break
    def readSerial(self):
        try:
            self.datain = self.ArdSerial.readline()
            self.datain = self.datain.decode('UTF-8')
            #print(self.datain)
            self.data = re.split('T|B|Ox|Oy|Oz|Ax|Ay|Az|M|N|P|Q', self.datain)
            del self.data[0]
            self.data[-1] = self.data[-1][:-2]
            if len(self.data) == 13:
                self.meas.update(self.data)
                self.meas.writeCSV(self.data)
        except UnicodeDecodeError: # catch unicode error and ignore it
            print('NaN Serial input error ')
    def writeSerial(self, outmsg):
        self.dataout = outmsg
        self.ArdSerial.write(self.dataout)

class Measurements(object):
    def __init__(self, gpsd):
        #create empty arrays to hold measurement data from arduino
	# "posterior" refers to the previous measurements
        
        self.orientation = np.zeros((3,1))
        self.orientationPosterior = np.zeros((3,1))
        self.omega = np.zeros((3,1))
        self.gpsd = gpsd
        self.linAccel = np.zeros((3,1))
        self.linAccelPosterior = np.zeros((3,1))
        self.avgAcc = np.zeros((3,1))
        
        self.rotation = np.zeros((3,3))
        
        
        self.linVel = np.zeros((2,1))
        self.linVelPosterior = np.zeros((2,1))
        self.delV = np.zeros((3,1))
        self.z= np.zeros((4,1))
        self.arduinoTime = 0
        self.arduinoTimePosterior = 0

        self.IMUposition = np.zeros((2,1))
        self.IMUpositionPosterior = np.zeros((2,1))
        self.motors = np.zeros((4,1))

        self.aMag = 0
        self.Vmag = 0
        
        self.heading = np.zeros((2,1))
        self.absHeading = 0;
        self.sonar = [0,0,0,0]
        self.voltage = 0.0
        self.counter = 0
        self.timestep = 0.1
        self.rollingavgAccel = []
        self.localAccel = np.zeros((2,1))
        self.localAccel3D = np.zeros((3,1))
        self.linVel3D = np.zeros((3,1))
        self.rollavg = 0
        self.GPS = [0.0, 0.0]
        self.GPSlocal = [0.0, 0.0]
        self.GPSlocallast = [0.0, 0.0]
        self.GPSerror = 0.0
        self.GPSdiff = [0.0, 0.0]
        self.GPSvel = 0.0
        self.GPSyaw = 0.0
        self.IMUdx = 0
        self.IMUdy = 0
        self.IMU3Dposition = np.zeros((3,1))
        self.refPosition = [0.0, 0.0]
        self.setReference([self.gpsd.fix.longitude, self.gpsd.fix.latitude])
        self.collideFlag = False
    def setReference(self, point):
        self.refPosition = point
    def update(self, data):
        #"data" is an 18 element list that is obtained from the serial input
        #this must be converted to numpy arrays and used to update the
        #measurement object variables
        self.raw = np.asarray(data)
        
        self.arduinoTime = float(self.raw[0])
        
        self.voltage = float(self.raw[1])
        #self.sonar[0] = int(self.raw[1])
        #self.sonar[1] = int(self.raw[2])
        #self.sonar[2] = int(self.raw[3])
        #self.sonar[3] = int(self.raw[4])
        
        
        if float(self.raw[2]) > 180:
            self.orientation[0] = float(self.raw[2])-360
        else:
            self.orientation[0] = float(self.raw[2])
        self.orientation[1] = float(self.raw[3])
        self.orientation[2] = float(self.raw[3])
        self.GPSlocallast = self.GPSlocal
        self.GPS = [float(self.gpsd.fix.longitude), float(self.gpsd.fix.latitude)]
        self.GPSlocal = GPS_single_point_transform(self.refPosition, self.GPS)
        self.GPSdiff[0] = self.GPSlocal[0] - self.GPSlocallast[0]
        self.GPSdiff[1] = self.GPSlocal[1] - self.GPSlocallast[1]
        self.GPSvel = magnitude(self.GPSdiff)
        self.GPSyaw = atan2(self.GPSdiff[1],self.GPSdiff[0])
        self.GPSerror = self.gpsd.fix.epx
        
         
        #self.rotation[0,0] = cos(self.orientation[0])*cos(self.orientation[1]) - sin(self.orientation[0])*sin(self.orientation[2])*sin(self.orientation[1])
        #self.rotation[0,1] = -sin(self.orientation[0])*cos(self.orientation[2])
        #self.rotation[0,2] = cos(self.orientation[0])*sin(self.orientation[1]) + sin(self.orientation[0])*sin(self.orientation[2])*cos(self.orientation[1])
        #self.rotation[1,0] = sin(self.orientation[0])*cos(self.orientation[1]) + cos(self.orientation[0])*sin(self.orientation[2])*sin(self.orientation[1])
        #self.rotation[1,1] = cos(self.orientation[0])*cos(self.orientation[2])
        #self.rotation[1,2] = sin(self.orientation[0])*sin(self.orientation[1]) - cos(self.orientation[0])*sin(self.orientation[2])*cos(self.orientation[1])
        #self.rotation[2,0] = -cos(self.orientation[1])*sin(self.orientation[2])
        #self.rotation[2,1] = sin(self.orientation[2])
        #self.rotation[2,2] = cos(self.orientation[2])*cos(self.orientation[1])
        
        self.linAccel[0] = float(self.raw[4])
        self.linAccel[1] = float(self.raw[5])
        self.linAccel[2] = float(self.raw[6])

        #self.localAccel[0,0] = -self.linAccel[0]*sin(radians(self.orientation[0])) - self.linAccel[1]*cos(radians(self.orientation[0]))
        #self.localAccel[1,0] = self.linAccel[0]*cos(radians(self.orientation[0])) - self.linAccel[1]*sin(radians(self.orientation[0]))
        self.localAccel[0,0] = self.linAccel[0]*cos(radians(self.orientation[0])) - self.linAccel[1]*sin(radians(self.orientation[0]))
        self.localAccel[1,0] = -self.linAccel[0]*sin(radians(self.orientation[0])) + self.linAccel[1]*cos(radians(self.orientation[0]))
        #self.localAccel3D = self.rotation.dot(self.linAccel)


        self.motors[0] = self.raw[7]
        self.motors[1] = self.raw[8]
        self.motors[2] = self.raw[9]
        self.motors[3] = self.raw[10]

        
        self.extrapolate()


    def extrapolate(self):
	# get timestep:
        if self.arduinoTimePosterior ==0:
            self.arduinoTimePosterior = self.arduinoTime
        else:
            self.timestep = (self.arduinoTime - self.arduinoTimePosterior)
            self.arduinoTimePosterior = self.arduinoTime
	# get average accel
        #self.avgAcc = (self.linAccel - self.linAccelPosterior)/2

	# get dphi/dt
        #self.omega = (self.orientation - self.orientationPosterior)/self.timestep
        #self.orientation = self.orientationPosterior

	#extrapolate velocity change
        #self.delV = self.avgAcc * self.timestep
	#extrapolate current velocity

        self.linVel = self.linVel + self.localAccel*self.timestep
        self.linVel3D = self.linVel3D + self.localAccel3D*self.timestep
        
            
        self.aMag = np.linalg.norm(self.linAccel)
        ##add new magnitude, then delete the oldest one after 4 have been appended
        #self.rollingavgAccel.append(self.aMag)
        self.counter += 1 
        #if self.counter > 2:
        #    del self.rollingavgAccel[0]
        #get average of the points in rollingavgAccel
        #self.rollavg = sum(self.rollingavgAccel)/len(self.rollingavgAccel)
        #if there is no baseline acceleration (rumble, vibrations), then it is likely that the robot is stopped. set velocity to zero.
        #if self.rollavg < 0.7:
        #    self.linVel.fill(0)
            
        self.vMag = np.linalg.norm(self.linVel)
        self.vMag2 = np.linalg.norm(self.linVel3D)
        #norm the velocity to 1 if it exceeds 1 (velocity limits)
        
        if self.vMag > 1:
            self.linVel = self.linVel/self.vMag
        if self.vMag2 > 1:
            self.linVel3D = self.linVel3D/self.vMag2
        #calculate position based on IMU:
        self.IMUdx = self.linVel[0,0]*self.timestep
        self.IMUdy = self.linVel[1,0]*self.timestep
        #axis, theta = [0,0,1], self.orientation[0]
        #M0 = M(axis, theta)

        #dot(M0,self.linAccel.T)
        #self.localAccel is IMUddx and IMUddy
        self.z[0,0] = self.linVel[0,0] #measured vel in x
        self.z[1,0] = self.linVel[1,0] # in y
        self.z[2,0] = self.localAccel[0,0] #measured accel in x
        self.z[3,0] = self.localAccel[1,0] # in y
        self.IMUposition[0,0] = self.IMUposition[0,0] + self.IMUdx
        self.IMUposition[1,0] = self.IMUposition[1,0] + self.IMUdy
        self.IMU3Dposition += self.linVel3D*self.timestep
        #self.linVelPosterior = self.linVel
	#extrapolate current position
	#(not using average velocity, instead using just last vel value.)

        
    def writeCSV(self, data):
        global filename
        with open(filename, 'a') as csvfile:
            csvLogger = csv.writer(csvfile)
            csvLogger.writerow(data)
    def writeCSVextra(self,data):
        global filename2
        with open(filename2, 'a') as csvfile:
            csvLogger = csv.writer(csvfile)
            csvLogger.writerow(data)
    def absHeading(self):
        self.heading[0] = magnet[0]*math.cos(self.orientation[0]/57.296) + magnet[1]*math.sin(self.orientation[2]/57.296)*math.sin(self.orientation[0]/57.296) - magnet[2]*math.cos(self.orientation[2]/57.296)*math.sin(self.orientation[0]/57.296)
        self.heading[1] = magnet[1]*math.cos(self.orientation[2]/57.296) - magnet[2]*math.sin(self.orientation[2]/57.296)
        self.absHeading = math.atan2(self.heading[1],self.heading[0])*57.296

class MotionControl(object):
        #The motion control object came about out of the need to perform path-following functions, PID, or even simple straight
        #line tests, which do not block the sensor reading / serial communications.
        #This is achieved through some video game programming methodology: have an
        #update function, run every cycle, which:
        #1. uses internal counter for time-dependent activities, and if statements to enact commands
        #2. provides a way to easily select and tweak the movement types of the device
        #3. is NON-BLOCKING, e.g. there are no loops or time-costly statements
        #this object will basically be the outgoing node for motor control -
        #it needs access to the arduino and measurement objects, so create
        #this object after those.
    def __init__(self, ard, meas):
        self.ard = ard
        self.meas = meas
        self.counter = 0
        self.mode = None
        self.rampSpeed1 = 0
        self.rampSpeed2 = 0
        self.flipflag = False
        self.doneflag = False
        self.Pcounter = 0
        self.readyFlag= False
    def pursuit(self, cx, cy, state):
        #first seek the first point in the array. turn until the robot is aligned with the vector direction of the start of the path.
        #then the robot will find the index of the nearest point (which will first be zero) then add maybe 3 or 4 to that number.
        #this is then the index of the point that the robot will seek. care must be taken not to have the robot seek a point which is way too
        #far ahead or at all behind of itself on the course, as this will result in strange behavior.
        #This method should allow the robot's possible speeds to govern how fast it will move along the path, rather than some fixed
        #velocity moving point (that may result in large error over time due to the point getting too far ahead of the bot). the motioncontrol
        #object's counter can be used, as long as it is not iterated twice in one loop.
        lookahead = 5
        #compare
            
        if self.Pcounter == 0:
            
            dx = cx[self.Pcounter]- state.x 
            dy = cy[self.Pcounter]- state.y 
            direct = atan2(dy, dx)
            magnit = magnitude([dx,dy])
            
            if magnit < 0.1:
                print("next index")
                self.Pcounter += lookahead
                
            return self.Pcounter, direct, magnit
        else:
            try:

                dx = cx[self.Pcounter]- state.x 
                dy = cy[self.Pcounter]- state.y

            except IndexError:
                dx = cx[-1]- state.x 
                dy = cy[-1]- state.y
                
            direct = atan2(dy, dx)
            magnit = magnitude([dx,dy])
            
            if magnit < 0.1:
                self.Pcounter += lookahead
                
            return self.Pcounter, direct, magnit
            
        
    def set_motors(self, speed1, speed2, speed3, speed4):
        #use 1 for forward, 0 for backward. set motor to 0 for stop, 255 for max speed. arduino handles motor calibration consts.
        #the arduino will send the motor state back upon setting the speed for confirmation.
        
        stringtemp1 =0
        stringtemp2 =0
        stringtemp3 =0
        stringtemp4 =0
        if speed1 > 0:
            stringtemp1 = '{:03d}'.format(speed1)
            stringtemp1 = '1' + stringtemp1
        else:
            stringtemp1 = '{:04d}'.format(speed1)
        if speed2 > 0:
            stringtemp2 = '{:03d}'.format(speed2)
            stringtemp2 = '1' + stringtemp2
        else:
            stringtemp2 = '{:04d}'.format(speed2)
        if speed3 > 0:
            stringtemp3 = '{:03d}'.format(speed3)
            stringtemp3 = '1' + stringtemp3
        else:
            stringtemp3 = '{:04d}'.format(speed3)
        if speed4 > 0:
            stringtemp4 = '{:03d}'.format(speed4)
            stringtemp4 = '1' + stringtemp4
        else:
            stringtemp4 = '{:04d}'.format(speed4)
        self.ard.writeSerial(('M%sN%sP%sQ%s'% (stringtemp1, stringtemp2, stringtemp3, stringtemp4)))


        
        #GPS poller runs in separate thread. accessing most current data is done by
        #using the gpsd.fix variable
    def motorMap(self, wR, wL):
        tRadius = 0.08
        vR = tRadius*wR
        vL = tRadius*wL
        leftPWM = 0
        rightPWM = 0
        #map left and right motor velocities to PWM speeds
        if vR > 1.15:
            vR = 1.15
        if vL > 1.15:
            vL = 1.15
        if vR < -1.15:
            vR = -1.15
        if vL < -1.15:
            vL = -1.15
        #assuming linear vel -> PWM mapping:
        leftPWM = vL*510/2.3
        rightPWM = vR*510/2.3
        return int(rightPWM), int(leftPWM)

    def motorTransform(self, omega):
        #from input omega, determine a linear velocity for each wheel.
        #first use omega as a proportional scaling factor to determine
        #the linear velocity
        #need to determine min and max omega and linvel through testing
        #MAX OMEGA (without negative or zero wheel velocity): 1.8 rad/s
        #
    
        framewidth = 0.413
        wheelrad = 0.08
        if omega > 1.4:
            omega = 1.4
        if omega < -1.4:
            omega = -1.4
        if not (0.000001 > omega > -0.000001): #float div by zero error protection
            lv = abs(0.12/omega) +0.5 #omega low: lv high. omega high, lv lower.
        #the number 0.12 allows omega from 0 to ~1.7
        #without any negative wheel velocity. 
        #0.5 means the lv will never drop below that number - so velocity should always be forward
        else:
            lv = 1
        #lv =1-(omega**2)/3.24
        
        #clamp lv between 0.5 and 1
        if lv > 1:
            lv = 1
        if lv < 0.5:
            lv = 0.5
    
        motormat= np.zeros((2,2))
        motormat[0,0] = wheelrad/2
        motormat[0,1] = wheelrad/2
        motormat[1,0] = -wheelrad/framewidth
        motormat[1,1] = wheelrad/framewidth
    
        solVect = np.zeros((2,1))
        solVect[0,0]= lv
        solVect[1,0] = omega
        omegaOut = np.linalg.solve(motormat, solVect)
    
        return float(omegaOut[0]), float(omegaOut[1]), lv

    def update(self, mode, parameter):
        #must select a method based on mode, which will run some function to control movement based on the counter (at 10 Hz)
        self.mode = mode
        if self.mode == 0:
            print("mode 0 - initialization")
            #initialization - fig 8 pattern, etc. 
        elif self.mode == 1:
            print("mode 1 - stationary turning test")
            #Test mode 1, stationary turning. Need to correlate motor speeds with angular velocity- lets set some commands.
            #setting at half speed one direction, then the other. 
            if (10 < self.counter < 80):
                self.ard.writeSerial(('M1250N0250P0000Q0000'))
                print('turnin em on')
               # self.set_motors(1125,0125,0125,0125)
            elif(85 < self.counter < 155):
                self.ard.writeSerial(('M0250N11251250P0000Q0000'))
               # self.set_motors(0125, 1125, 0125, 0125)
                print('turnin em the other way')
            else:
                self.ard.writeSerial(('M0000N0000P0000Q0000'))
                print('off')
                #self.set_motors(0, 0, 0, 0)
        elif self.mode == 2:
            print('straight line mode')
            
            
            if (10 < self.counter < 80):
                self.ard.writeSerial(('M1125N1125P0000Q0000'))

               # self.set_motors(1125,0125,0125,0125)
            elif(85 < self.counter < 155):
                self.ard.writeSerial(('M0125N0125P0000Q0000'))
               # self.set_motors(0125, 1125, 0125, 0125)
            else:
                self.ard.writeSerial(('M0000N0000P0000Q0000'))
                print('off')
            #test, straight line
        elif self.mode == 3:
            #for padding zeros:
              #speed increase by 1 every 2 cycles (motor
            #update rate = 5 Hz)
            if not self.doneflag:
                if not self.flipflag:
                    self.rampSpeed1 += 5            
                    if self.rampSpeed1 > 254:
                        self.rampSpeed1 = 255
                        self.flipflag = True
                    
                else:
                    self.rampSpeed1 -= 5
                    if self.rampSpeed1 < -254:
                        self.flipflag = False
                        self.rampSpeed1 = -255

                if self.rampSpeed1 > 0:
                    stringtemp = '{:03d}'.format(self.rampSpeed1)
                    stringtemp = '1' + stringtemp
                else:
                    stringtemp = '{:04d}'.format(-self.rampSpeed1)
                if self.counter > 300:
                    self.doneflag = True
            
            else:
                stringtemp = '0000'
            #print(stringtemp)
            self.ard.writeSerial(('M%sN%sP0000Q0000'% (stringtemp, stringtemp)))
           # print('{:03d}'.format(n))

        #speed ramp test - steady increase + decrease in velocity.
        elif self.mode == 4:
            #non-stationary turning: both wheels at some forward velocity.
            #angular velocity is some function of 
            
            print('nothin yet')
            
        self.counter += 1
class PID(object):
    def __init__(self, state):
        #ziegler nichols:
        ##current implementation: PI --> 
        #Ku = 2.22
        #Tu = 0.7s
        #yields KP = 1, KI = 0.58 (Tu/1.2)
        self.P = 0.7
        self.I = 0.4
        self.D = 0
        self.state = state
        self.error = 0
        self.cHeading = state.yaw
        self.deltaT = 0.1
        self.output = 0
        self.integral = 0
        self.errorlast = 0
    def update(self, dHeading):
        #the kalman filter object should be run prior to reading this state
        
        self.cHeading = self.state.yaw
        self.error = dHeading - self.cHeading
        if (self.error > 180):
            self.error = self.error - 360
        elif (self.error < -180):
            self.error = self.error +360
        else:
            pass
        self.integral = self.integral +radians(self.error)*self.deltaT
        if self.integral > 5:
            self.integral = 5
        if self.integral < -5:
            self.integral = -5
        self.output = self.P * radians(self.error) + self.I*self.integral
        self.errorlast = self.error
        return self.output
class GpsPoller(threading.Thread):
  def __init__(self):
    threading.Thread.__init__(self)
    global gpsd #bring it in scope
    gpsd = gps(mode=WATCH_ENABLE) #starting the stream of info
    self.current_value = None
    self.running = True #setting the thread running to true
 
  def run(self):
    global gpsd
    while gpsp.running:
        gpsd.next() #this will continue to loop and grab EACH set of gpsd info to clear the buffer
class Spline:
    u"""
    Cubic Spline class
    """
    # This is for ONE Dimension e.g. x(t)
    # http://math.oregonstate.edu/~restrepo/475A/Notes/sourcea-/node35.html#cubiccc
    def __init__(self, x, y):
        #coefficients for the control points?
        self.b, self.c, self.d, self.w = [], [], [], []
        #control points are x and y (each 1D numpy arrays)
        self.x = x
        self.y = y

        self.nx = len(x)  # dimension of x
        #diff calculates the difference between x adjacent elements in the array, and stores them in x - 1 spaces in an output array
        h = np.diff(x)
        #basically, the length of the lines between the control points

        # calc coefficient c - 'a' vector seems to store the y points
        self.a = [iy for iy in y]
        
        # calc coefficient c
        A = self.__calc_A(h)
        B = self.__calc_B(h)
        self.c = np.linalg.solve(A, B)
        #solves for the spline coefficient
        #  print(self.c1)

        # calc spline coefficient b and d - clamped case, z = 2 for b
        for i in range(self.nx - 1):
            self.d.append((self.c[i + 1] - self.c[i]) / (3.0 * h[i]))
            tb = (self.a[i + 1] - self.a[i])/h[i] - h[i]*(self.c[i + 1] + 2.0*self.c[i])/3.0
            self.b.append(tb)

    def calc(self, t):
        u"""
        Calc y position from input x location (here t)

        if t is outside of the input x, return None

        """

        if t < self.x[0]:
            return None
        elif t > self.x[-1]:
            return None
        #find point to the right of the interval the input x fits into of the input point vector
        i = self.__search_index(t)
        #calculate the distance to the x at that index from t
        dx = t - self.x[i]
        #result is combination of y value at that index (a), dx, and spline coefficients b, c and d, calculated above
        result = self.a[i] + self.b[i]*dx+self.c[i] * dx ** 2.0 + self.d[i] * dx ** 3.0
        #i believe result is simply the y-value for the input x value (t)
        return result

    def calcd(self, t):
        u"""
        Calc first derivative of y position (tangent) from input x location

        if t is outside of the input x, return None
        """

        if t < self.x[0]:
            return None
        elif t > self.x[-1]:
            return None

        i = self.__search_index(t)
        dx = t - self.x[i]
        result = self.b[i] + 2.0 * self.c[i] * dx + 3.0 * self.d[i] * dx ** 2.0
        return result

    def calcdd(self, t):
        u"""
        Calc second derivative of y position (curvature) from input x location
        """

        if t < self.x[0]:
            return None
        elif t > self.x[-1]:
            return None

        i = self.__search_index(t)
        dx = t - self.x[i]
        result = 2.0 * self.c[i] + 6.0 * self.d[i] * dx
        return result

    def __search_index(self, x):
        u"""
        search data segment index
        """
        #bisect.bisect function returns where the element x would fit in the sorted list self.x. (which interval it's in)
        return bisect.bisect(self.x, x) - 1

    def __calc_A(self, h):
        u"""
        calc matrix A for spline coefficient c (stiffness matrix)
        """
        A = np.zeros((self.nx, self.nx)) # size of x dimension
        A[0, 0] = 1.0 #setting first element as 1 for some reason
        for i in range(self.nx - 1): #use nx-1 now, calculations are based on interaction between adjacent points, not points themselves
            if i != (self.nx - 2): #dont do this for the second to last element
                A[i + 1, i + 1] = 2.0 * (h[i] + h[i + 1]) #filling out diagonals of the matrix: equal to 2 times the height plus the length of that element plus the length of the next. 
            A[i + 1, i] = h[i] # the two diagonal-line entries adjacent to the diagonal both set as the length vector entries
            A[i, i + 1] = h[i]

        A[0, 1] = 0.0 
        A[self.nx - 1, self.nx - 2] = 0.0 
        A[self.nx - 1, self.nx - 1] = 1.0
        #  print(A)
        return A

    def __calc_B(self, h):
        u"""
        calc matrix B for spline coefficient c (forcing/ boundary conditions matrix)
        """
        B = np.zeros(self.nx)
        for i in range(self.nx - 2):
            B[i + 1] = 3.0 * (self.a[i + 2] - self.a[i + 1])/h[i + 1] - 3.0 * (self.a[i + 1] - self.a[i])/h[i]
        #  print(B)
        return B


class Spline2D:
    """
    2D Cubic Spline class
    This class basically just combines two 1D splines (simple polynomial one-to-one functions) in order to have full 2d parametric representations (which may or may not be one-to-one)
    """

    def __init__(self, x, y):
        #get rough arc length
        self.s = self.__calc_s(x, y)
        #create two 1-d splines for x and y projections. one dimension held const for each (self.s)
        self.sx = Spline(self.s, x)
        self.sy = Spline(self.s, y)
        
    def __calc_s(self, x, y):
        """
        calculate the rough arc length given x and y vectors
        """
        #get intervals hx and hy
        dx = np.diff(x)
        dy = np.diff(y)
        
        self.ds = [sqrt(idx ** 2 + idy ** 2) for (idx, idy) in zip(dx, dy)]
        s = [0] #add zero to list s
        #extend is like append but it appends multiple objects. 
        s.extend(np.cumsum(self.ds)) #cumulative sum is basically addition factorial, stored as successive values in a list.
        # the last element is the total length calculated using linear functions between input points
        return s

    def calc_position(self, s):
        u"""
        calc position
        just forwards this to the individual 1-d spline's position calc function
        """
        x = self.sx.calc(s)
        y = self.sy.calc(s)

        return x, y

    def calc_curvature(self, s):
        u"""
        calc curvature
        just forwards this to the individual 1-d spline's deriv calc functions
        """
        dx = self.sx.calcd(s)
        ddx = self.sx.calcdd(s)
        dy = self.sy.calcd(s)
        ddy = self.sy.calcdd(s)
        k = (ddy * dx - ddx * dy) / (dx ** 2 + dy ** 2)
        return k

    def calc_yaw(self, s):
        u"""
        calc yaw (tangent angle)
        """
        dx = self.sx.calcd(s)
        dy = self.sy.calcd(s)
        yaw = atan2(dy, dx)
        return yaw
def calc_target_index(state, cx, cy):

    # search nearest point index
    dx = [state.x - icx for icx in cx]
    dy = [state.y - icy for icy in cy]
    d = [abs(sqrt(idx ** 2 + idy ** 2)) for (idx, idy) in zip(dx, dy)]
    ind = d.index(min(d))
    L = 0.0

    Lf = k * state.v + Lfc

    # search look ahead target point index
    while Lf > L and (ind + 1) < len(cx):
        dx = cx[ind + 1] - cx[ind]
        dy = cx[ind + 1] - cx[ind]
        L += sqrt(dx ** 2 + dy ** 2)
        ind += 1

    return ind
def GPS_transform(refcoordinate, pointarray):
    #transforms a set GPS coordinates to a local reference frame, based on latitude and one 
    #reference point. 
    #coordinate 0 - lat, coordinate 1 -long
    #beta term accounts for ellipsoidal earth
    beta = atan(tan(radians(refcoordinate[0]))*0.99664719)
    #longitude and latitude scaling factors
    longitudeMeters = (np.pi/180)*6367449*cos(beta)
    latitudeMeters = 110845.0
    #get dx and dy from reference point to point array
    dx = []
    dy = []
    dxm = []
    dym = []
    for point in pointarray:
        dx.append((point[0] - refcoordinate[0])*longitudeMeters)
        dy.append((point[1] - refcoordinate[1])*latitudeMeters)
    return dx, dy, longitudeMeters, latitudeMeters
def GPS_single_point_transform(refcoordinate, tPoint):
    beta = atan(tan(radians(refcoordinate[0]))*0.99664719)
    longitudeMeters = (np.pi/180.0)*6367449.0*cos(beta)
    latitudeMeters = 110845.0
    dx = (tPoint[0] - refcoordinate[0])*longitudeMeters
    dy = (tPoint[1] - refcoordinate[1])*latitudeMeters
    return [dx, dy]
def haversine(lon1, lat1, lon2, lat2):
    #gets distance between two gps points
    # convert decimal degrees to radians 
    lon1, lat1, lon2, lat2 = map(radians, [lon1, lat1, lon2, lat2])
    # haversine formula 
    dlon = lon2 - lon1 
    dlat = lat2 - lat1 
    a = sin(dlat/2)**2 + cos(lat1) * cos(lat2) * sin(dlon/2)**2
    c = 2 * asin(sqrt(a)) 
    # Radius of earth in meters is 6371000
    m = 6371000* c
    return m

def magnitude(vector):
    #simple 2d vector magnitude
    return (vector[0] ** 2 + vector[1] ** 2) ** 0.5
def rotate(l, n):
    #function for "rolling" or "rotating" a list by removing a number of elements
    #from the rear of the list and putting them at the front
    return l[n:] + l[:n]

def trim(sonarvals):
    #sets the 200 value to 0, to make it easier to eliminate false positive readings. all 0 readings will be ignored.
    if sonarvals[0] == 200:
        sonarvals[0] = 0
    if sonarvals[1] == 200:
        sonarvals[1] = 0
    if sonarvals[2] == 200:
        sonarvals[2] = 0

def raycast(sonarvals):
    
    xa = -sonarvals[0]*.7071
    ya = sonarvals[0]*.7071
    xb = 0
    yb = sonarvals[1]
    xc = sonarvals[2]*.7071
    yc = sonarvals[2]*.7071
    return [(xa, ya), (xb, yb), (xc, yc)]
def raycastLocal(sonarvals, position, orientation):
    #sonarvals: trimmed sonar values in cm
    xa = position[0] + sonarvals[0]*cos(radians(orientation-45))/100
    ya = position[1] - sonarvals[0]*sin(radians(orientation-45))/100
    xb = position[0] + sonarvals[1]*cos(radians(orientation))/100
    yb = position[1] - sonarvals[1]*sin(radians(orientation))/100
    xc = position[0] + sonarvals[2]*cos(radians(orientation+45))/100
    yc = position[1] - sonarvals[2]*sin(radians(orientation+45))/100
    return [(xa, ya), (xb,yb), (xc,yc)]

def testCommandParse(commandstring):
    try:
        testTime = int(commandstring[5:8])
    except ValueError:
        testTime = 0
        pass
    testType = 0
    val = 0
    try:
        if commandstring[0] == "F":
            testType = 1
            val = int(commandstring[1:4])
            print("forwared")
        
        elif commandstring[0] == "R":
            testType = 2
            val = int(commandstring[1:4])
            print("reverse")
        
        elif commandstring[0] == "T":
            testType = 3
            val = float(commandstring[1:4])
            print("turn")
        elif commandstring[0] == "P":
            testType = 4
            val = int(commandstring[1:4])
            print("PID")
        elif commandstring[0] == "S":
            testType = 5
            val = int(commandstring[1:4])
            print("Sonar-aware PID")

        else:
            print("nocommand")
    except:
        print('no command, sensor read only mode')
        pass
    print(val)
    return val, testTime, testType
class Kalman(object):
    def __init__(self, state, meas):
        self.state= state
        self.meas = meas
        #state transition error
        #how much error can occur in 0.1s? - est: 1 m/s (meas) - 0.8m/s(actual) = 0.2 m/s
        #0.2 m/s * 0.1 s = 0.02
        self.Q = np.asmatrix(np.eye(4)*0.02)
        #state transition constant - no position or velocity change without control input
        self.A = np.asmatrix(np.eye(4))
        #B is control matrix, translates control vector to state vector units
        self.B = np.asmatrix(np.eye(4)*self.meas.timestep)
        #R is measurement error matrix (pretty high error for now)
        self.R = np.asmatrix(np.eye(4)*0.5)
        self.P = np.asmatrix(np.eye(4))

        self.xlast = np.asmatrix(np.zeros((4,1)))
        self.x = np.asmatrix(np.zeros((4,1)))
        self.Un = np.asmatrix(np.zeros((4,1)))
        self.UnLast = np.asmatrix(np.zeros((4,1)))
        self.xpred = np.asmatrix(np.zeros((4,1)))
        self.Ppred = np.asmatrix(np.zeros(4))
        self.Plast = np.asmatrix(np.zeros(4))
        self.innov = np.asmatrix(np.zeros((4,1)))
        self.H = np.asmatrix(np.eye(4))
        self.S = np.asmatrix(np.zeros((4,1)))
        self.K = np.asmatrix(np.zeros((4)))
    def predict(self):
        #get new control vector - 
        #first two elements of the control vector are calculated based on dead reckoning velocity
        #and known yaw (projected into local reference frame)
        self.Un[0,0]= float(self.state.DRv) * cos(radians(self.state.yaw))*(0.2)
        self.Un[1,0] = -float(self.state.DRv) * sin(radians(self.state.yaw))*(0.2)
        #the next two have to be calculated by saving the two velocity values and dividing by the timestep (0.1s)

        self.Un[2,0] = (self.Un[0,0] - self.UnLast[0,0])/self.meas.timestep
        self.Un[3,0] = (self.Un[1,0] - self.UnLast[1,0])/self.meas.timestep
        #self.Un = np.asmatrix(self.Un)
        #set new value to old
        self.UnLast = self.Un
        #state prediction
        self.xpred = self.A*self.xlast + self.B*self.Un
        #covariance prediction
        self.Ppred = self.Plast + self.Q

    def update(self):
        #compare dead reckoning state estimation with IMU state estimation:
        self.innov = self.meas.z -self.H*self.xpred
        #innovation covariance
        self.S = self.Ppred + self.R
        #Kalman Gain
        self.K = self.Ppred*np.linalg.inv(self.S)
        # state update:

        self.x = self.xpred + self.K*self.innov
        # covariance update:
        self.P = (np.eye(4)- self.K*self.H)*self.Ppred
        #set new to old variables:
        self.xlast = self.x
        self.Plast = self.P
        
def course_diversion(sonarmeans, sonarpoints, currentyaw):
    #sonarpoints is a set of locally transformed sonar points based on the mean of several readings
    #first, see which sensors have readings.
    #[(xa, ya), (xb,yb), (xc,yc)]
    sortsonar = sorted(sonarmeans)
    print(sonarmeans)
    print(sortsonar)
    firstp = (0, 0)
    secondp = (0, 0)
    
    if sortsonar[0] is 0:
        #one reading is 0: at most 2 readings are valid
        if sortsonar[1] is 0:
            if sortsonar[2] is 0:
                #no readings - return output yaw the same as current yaw
                outputyaw = currentyaw
            else:
                if sonarmeans[0] is not 0:
                    #left sensor hit: turn right 45 deg
                    outputyaw = currentyaw + 45.0
                elif sonarmeans[1] is not 0:
                    #center: turn right 60 degrees
                    outputyaw = currentyaw + 60.0
                elif sonarmeans[2] is not 0:
                    #right: turn left 45 degrees 
                    outputyaw = currentyaw - 45.0
                #exactly one reading
            #two readings are zero: at most 1 reading is valid
        else:
            #exactly two readings
            if sortsonar[0] == sonarmeans[0]:
                firstp = sonarpoints[0]
            elif sortsonar[0] == sonarmeans[1]:
                firstp = sonarpoints[1]
            elif sortsonar[0] == sonarmeans[2]:
                firstp = sonarpoints[2]
            else:
                print('cant find closest point')
            if sortsonar[1] == sonarmeans[0]:
                secondp = sonarpoints[0]
            elif sortsonar[1] == sonarmeans[1]:
                secondp = sonarpoints[1]
            elif sortsonar[1] == sonarmeans[2]:
                secondp = sonarpoints[2]
            else:
                print('cant find second point')
                
    else:
        #exactly three readings
        if sortsonar[0] == sonarmeans[0]:
            firstp = sonarpoints[0]
        elif sortsonar[0] == sonarmeans[1]:
            firstp = sonarpoints[1]
        elif sortsonar[0] == sonarmeans[2]:
            firstp = sonarpoints[2]
        else:
            print('cant find closest point')
        if sortsonar[1] == sonarmeans[0]:
            secondp = sonarpoints[0]
        elif sortsonar[1] == sonarmeans[1]:
            secondp = sonarpoints[1]
        elif sortsonar[1] == sonarmeans[2]:
            secondp = sonarpoints[2]
        else:
            print('cant find second point')

    #now that the closest points have been determined, its time to set the output yaw:
    if firstp is not (0,0):
        outputyaw = degrees(atan2((secondp[1] - firstp[1]), (secondp[0] - firstp[0])))
        
    return outputyaw
        
def main():
    #logging.getlogger(debugLog)
    connectflag = True
    loopflag = True
    stopflag = False
    STOP_RANGE = 100
    STOP_RANGE2 = 1
    STOP_TIMER = 20
    divertflag = False
    DIVERT_TIMER = 20
    global gpsd
    try:
        gpsp.start() #DISABLE FOR PC TESTING - (NO GPS)
        sleep(1)
    except:
        print('no GPS connection')
        pass
    while connectflag:
        print "Waiting for connection on RFCOMM channel %d" % port

        client_sock, client_info = server_sock.accept()
        print "Accepted connection from ", client_info
        sleep(1)
        try:
            #data is buffer that stores bluetooth incoming transmission data
            # i think this is blocking - so the program will wait here until
            # something is received.
            incomingData = client_sock.recv(1024)
            if len(incomingData) == 0:
                print('nada')
                break
            print "received [%s]" % incomingData
            connectflag = False

        
            #client_sock.send(data)
        #print "sending [%s]" % data

        except IOError as e:
            print(e)
            connectflag = False
            
        except KeyboardInterrupt:
            print "disconnected"
            connectflag = False
            
        client_sock.close()
        server_sock.close()
        connectflag = False
    tempvar = re.split('X|Y|E|D', incomingData)
    del tempvar[0]
    coordinates = tempvar[0:-3]
    extra = tempvar[-2]
    print(extra)
    
    cInd = 0
    
    first = True
    on = True
    previousMillis = 0
    previousMillis2 = 0
    #filename = 'test.csv'
    # opening the file with w+ mode resets the file
    global filename
    f = open(filename, "w+")
    f.close()
    global filename2
    f = open(filename2, "w+")
    f.close()
    
    
    meas = Measurements(gpsd)

    
    xinit = 0
    yinit = 0
    state = State(xinit, yinit, 0, 0)
    pid = PID(state)
    kal = Kalman(state, meas)
    xproj = 0
    yproj = 0
    out_angle = 0
    dataout = []
    deadReckonx = 0
    deadReckony = 0
    bodySonar = []
    sonarList1= []
    sonarList2 = []
    sonarList3 = []
    sonarList4 = []
    sonarBestGuess = [0,0,0,0]
    val, testTime, testType = testCommandParse(extra)

    print('test type', testType)
    print('received Value:', val)
    try:
                
        try:
                    
            ard = Arduino(meas)
            ard.connectArduino()   
            print('Connected')
        except:
            print('Could not connect')
            on = False
        mControl = MotionControl(ard, meas)
        #courseindex = -3
        #print(len(ryaw), 'course step length')
        while on:
            
            ms = time() * 1000

            ard.readSerial()
            
                
            if (ms - previousMillis > interval):
                
                previousMillis = ms

                try:
                    state.v = state.v[0]
                except:
                    #print('lv is int')
                    pass
                try:
                    state.yaw = meas.orientation[0,0]
                except:
                    state.yaw = meas.orientation[0]
                    #print('orientation is not array')
                    pass
                trim(meas.sonar)
                #get dead reckoning
                #print(state.DRv)
                #print(state.yaw)
                #print(xproj)
                #print(yproj)
                xproj = float(state.DRv) * cos(radians(state.yaw))*(0.1)
                yproj = -float(state.DRv) * sin(radians(state.yaw))*(0.1)
                deadReckonx = deadReckonx + xproj
                deadReckony = deadReckony + yproj
                #set state - dead reckoning for now, kalman filtered soon. orientation from filtered output of IMU.
                state.x = deadReckonx 
                state.y = deadReckony
                #state.yaw = meas.orientation[0]
                kal.predict()
                kal.update()
                
                
                    
                #sonarList1.append(meas.sonar[0])
                #sonarList2.append(meas.sonar[1])
                #sonarList3.append(meas.sonar[2])
                #sonarList4.append(meas.sonar[3])
                #print(sonarList1)
                #stdarray = np.array([sonarList1, sonarList2, sonarList3, sonarList4])
                #print(stdarray)
                
                #if len(sonarList1) > 1:
                    #if the difference between the last two readings is within 25cm, then the reading sticks, otherwise, use the last reading.
                    #this should eliminate single-reading spikes. basically, it just requires that two adjacent-in-time points have at least a
                    #somewhat similar reading, otherwise the reading will not change.
                    #if (sonarList1[1]-sonarList1[0]) < 25:
                        
                    #    sonarBestGuess[0] = sonarList1[1]
                        
                        
                    #if (sonarList2[1]-sonarList2[0]) < 25:
                        
                    #    sonarBestGuess[1] = sonarList2[1]

                        
                    #if (sonarList3[1]-sonarList3[0]) < 25:
                        
                    #	sonarBestGuess[2] = sonarList3[1]

                        
                    #if (sonarList4[1]-sonarList4[0]) < 25:
                        
                    #    sonarBestGuess[3] = sonarList4[1]

                    
                    #stdev = np.std(stdarray, axis=1)
                    #sonarmean = np.mean(stdarray, axis=1)
                #else:
                    #sonarBestGuess[0] = 0
                    #sonarBestGuess[1] = 0
                    #sonarBestGuess[2] = 0
                    #sonarBestGuess[3] = 0
                    
                
                #if len(sonarList1) > 2:
                    #del sonarList1[0]
                #if len(sonarList2) > 2:
                    #del sonarList2[0]
                #if len(sonarList3) > 2:
                    #del sonarList3[0]
                #if len(sonarList4) > 2:
                    #del sonarList4[0]
                
                
                #bodySonar = raycast(sonarBestGuess)
                #localSonar = raycastLocal(sonarBestGuess, [state.x, state.y], state.yaw)
                
                dataout.append(kal.x[0,0])
                dataout.append(kal.x[1,0])
                dataout.append(meas.IMUposition[0,0])
                dataout.append(meas.IMUposition[1,0])
                #dataout.append(meas.localAccel3D[0,0])
                #dataout.append(meas.localAccel3D[1,0])
                #dataout.append(meas.localAccel3D[2,0])
                dataout.append(float(deadReckonx))
                dataout.append(float(deadReckony))
                dataout.append(meas.GPSlocal[0])
                dataout.append(meas.GPSlocal[1])
                
                #dataout.append(localSonar[0][0])
                #dataout.append(localSonar[0][1])
                #dataout.append(localSonar[1][0])
                #dataout.append(localSonar[1][1])
                #dataout.append(localSonar[2][0])
                #dataout.append(localSonar[2][1])
                #dataout.append(meas.sonar[0])
                #dataout.append(meas.sonar[1])
                #dataout.append(meas.sonar[2])
                #dataout.append(meas.sonar[3])
                #dataout.append(sonarBestGuess[0])
                #dataout.append(sonarBestGuess[1])
                #dataout.append(sonarBestGuess[2])
                #dataout.append(sonarBestGuess[3])
                dataout.append(state.yaw)
                #dataout.append(out_angle)
                #dataout.append(stdev[0])
                #dataout.append(stdev[1])
                #dataout.append(stdev[2])
                #dataout.append(stdev[3])
				dataout.append(meas.voltage)
                meas.writeCSVextra(dataout)
                del dataout[:]
            if (ms - previousMillis2 > interval2):
                previousMillis2 = ms
                
                
                #print(stdev)

                #cInd, cDir, cMag = mControl.pursuit(rx, ry, state) 
                #m1, m2, lv = mControl.motorTransform(pid.update(45))
                #m1, m2, lv = mControl.motorTransform(pid.update(cDir*57))
                #PWM1, PWM2 = mControl.motorMap(m1, m2)
                #print(meas.GPS)
                
                #mControl.set_motors(PWM1, PWM2, 0,0)
                #motor configuration:
                #(B, C, L, R)
                #POSITIVE IS FORWARD for all

                #activate different test routines based on text input from user application
                
                if testType == 1:
                    #forward
                    if mControl.counter < testTime:
                        print(val)
                        print(type(val))
                        print(val/255.0)
                        
                        mControl.set_motors(0,0,val, val)
                        state.DRv = float(val/225.0)
                        print(state.DRv)
                        mControl.counter +=1
                    else:
                        mControl.set_motors(0,0,0,0)
                        state.DRv = 0
                        break
                elif testType == 2:
                    if mControl.counter < testTime:
                        mControl.set_motors(0,0,-val, -val)
                        state.DRv = -float(val/225.0)
                        mControl.counter +=1
                    else:
                        mControl.set_motors(0,0,0,0)
                        state.DRv = 0
                        break
                elif testType == 3:
                    if mControl.counter < testTime:
                        if ((meas.sonar[0] > STOP_RANGE) or (meas.sonar[0] == 0)) and ((meas.sonar[1] > STOP_RANGE) or (meas.sonar[1] == 0)) and ((meas.sonar[2] > STOP_RANGE) or (meas.sonar[2] == 0)):
                            cInd, cDir, cMag = mControl.pursuit(rx, ry, state)
                            m1, m2, state.DRv = mControl.motorTransform(pid.update(cDir*57.0))
                        
                            PWM1, PWM2 = mControl.motorMap(m1, m2)
                            mControl.set_motors(0, 0, PWM2, PWM1)
                            mControl.counter +=1
                        else:
                            mControl.set_motors(0,0,0,0)
                            state.DRv = 0
                    else:
                        mControl.set_motors(0,0,0,0)
                        state.DRv = 0
                        break
                elif testType == 4:
                    
                    if mControl.counter < testTime:
                        
                        m1, m2, state.DRv = mControl.motorTransform(pid.update(val))
                        PWM1, PWM2 = mControl.motorMap(m1, m2)
                        mControl.set_motors(0, 0, PWM2, PWM1)
                        mControl.counter +=1
                    else:
                        mControl.set_motors(0,0,0,0)
                        state.DRv = 0
                        break
                elif testType == 5:
                    if mControl.counter < testTime:
                        if stopflag is False:
                            if ((1 < meas.sonar[0] < STOP_RANGE) or (1 < meas.sonar[1] < STOP_RANGE) or (1 < meas.sonar[2] < STOP_RANGE)):
                                #if sonar detects object within range 1 (70cm) it will stop and do a "sanity check" where it gets the mean of the sonar readings over a period
                        #These sonar means will be used to determine the course deflection angle. once the stop timer is exceeded, the angle will be chosen and a new
                        #stop range of 40 cm will be used to allow the robot to turn away from the obstacle without stopping again
                                stopflag = True
                                divertflag = False
                                
                        if stopflag:
                            if STOP_TIMER > 0:
                                STOP_TIMER -= 1
                                divertflag = False
                                mControl.set_motors(0,0,0,0)
                                state.DRv = 0.0
                            else:
                            
                                stopflag = False
                                divertflag = True
                                STOP_TIMER = 20
                          
                        if divertflag:
                            if DIVERT_TIMER == 10:
                                out_angle = course_diversion(sonarBestGuess[:3], bodySonar, state.yaw)
                            if DIVERT_TIMER > 0:
                                DIVERT_TIMER -= 1
                                if ((meas.sonar[0] > STOP_RANGE2) or (meas.sonar[0] == 0)) and ((meas.sonar[1] > STOP_RANGE2) or (meas.sonar[1] == 0)) and ((meas.sonar[2] > STOP_RANGE2) or (meas.sonar[2] == 0)):
                                
                                    m1, m2, state.DRv = mControl.motorTransform(pid.update(out_angle))
                                    PWM1, PWM2 = mControl.motorMap(m1, m2)
                                    mControl.set_motors(CB_speed, BW_speed, PWM1, PWM2)
                                else:
                                    mControl.set_motors(0,0,0,0)
                                    state.DRv = 0.0

                            else:
                                divertflag = False
                                DIVERT_TIMER = 10
                            
                        if not stopflag or divertflag:
                            m1, m2, state.DRv = mControl.motorTransform(pid.update(val))
                            PWM1, PWM2 = mControl.motorMap(m1, m2)
                            mControl.set_motors(CB_speed, BW_speed, PWM1, PWM2)
                            mControl.counter +=1
                        else:
                            mControl.set_motors(0,0,0,0)
                            state.DRv = 0.0
                            
                else:
                    pass
              

                
    except KeyboardInterrupt:
        print("\nKilling motors")
        mControl.set_motors(0,0,0,0)
    print("\nKilling GPS thread")
    gpsp.running = False #DISABLE FOR PC TESTING - (NO GPS)

    gpsp.join() # wait for the thread to finish what it's doing DISABLE FOR PC TESTING - (NO GPS)

    print("ending sensor read")


if __name__ == "__main__":

    interval = 100
    interval2 = 200
    gpsp = GpsPoller() #DISABLE FOR PC TESTING - (NO GPS)
    
    main()

