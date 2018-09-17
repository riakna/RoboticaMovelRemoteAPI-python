# -*- coding: utf-8 -*-
"""
Created on Wed Sep 12 10:34:58 2018

@author: Anderson
"""

import numpy as np
from simulator import Simulator
import time

class Robot:

    # Handles 
    robotHandle = None
    motorHandle = [None] * 2
    wheelHandle = [None] * 2
    sonarHandle = [None] * 16    
    
    # Ground Truths
    robotPosOrn = {
            'first': [None] * 3,
            'current':  [None] * 3,
            'last': [None] * 3
    }
    
    # Odometry
    odometryPosOrn = {
            'raw': [None] * 3,
            'compass':  [None] * 3
    }
    lastTime = None
    
    
    # Constants
    SONAR_POSITIONS = [None] * 16   
    PI = np.pi
    SONAR_ANGLES = np.array([PI/2, 50/180.0*PI, 30/180.0*PI, 10/180.0*PI, 
                       -10/180.0*PI, -30/180.0*PI, -50/180.0*PI, -PI/2, -PI/2, 
                       -130/180.0*PI, -150/180.0*PI, -170/180.0*PI, 170/180.0*PI,
                       150/180.0*PI, 130/180.0*PI, PI/2]) 
    
    L = None
    R = None
    
    # L = 0.3309488597888182
    # R = 0.09749898314476013
    
    def __init__(self):

        self.sim = Simulator('127.0.0.1', 25000)
        self.sim.connect()        
        ### -----------
        ### GET HANDLES

        # ROBOT
        self.robotHandle = self.sim.getHandle("Pioneer_p3dx")
        
        # ACTUATORS
        self.motorHandle[0] = self.sim.getHandle('Pioneer_p3dx_leftMotor')
        self.motorHandle[1] = self.sim.getHandle('Pioneer_p3dx_rightMotor')
    
        # SONARS
        for x in range(0, len(self.sonarHandle)):
            self.sonarHandle[x] = self.sim.getHandle('Pioneer_p3dx_ultrasonicSensor'+str(x+1))
            
        # VISION
        self.visionHandle = self.sim.getHandle('Pioneer_p3dx_rightMotor')
            
        # WHEEL
        self.wheelHandle[0] = self.sim.getHandle('Pioneer_p3dx_leftWheel')
        self.wheelHandle[1] = self.sim.getHandle('Pioneer_p3dx_rightWheel')
        
        
        ### -----------
        ### INIT STREAMS
        self.sim.initObjectPositionStream(self.robotHandle, -1)
        self.sim.initObjectOrientationStream(self.robotHandle, -1)
        for x in range(0, len(self.sonarHandle)):
            self.sim.initProximitySensor(self.sonarHandle[x])
        self.sim.initFloatSignal("gyroZ")
        self.sim.initVisionSensorImage(self.visionHandle)
        self.sim.initObjectVelocity(self.robotHandle)
        self.sim.initJointPositionStream(self.motorHandle[0])
        self.sim.initJointPositionStream(self.motorHandle[1])
        
        
        
        ### -----------
        ### POSTIONS    
        
        # SONAR (em relacao ao frame do robo)
        for x in range(0, len(self.sonarHandle)):
            self.SONAR_POSITIONS[x] = self.sim.getObjectPositionBlock(
                    self.sonarHandle[x], self.robotHandle) 
            
        self.L = np.linalg.norm(self.sim.getObjectPositionBlock(
                self.wheelHandle[0], self.wheelHandle[1]))
        
        #self.L = 0.36205;
        
        # Robo precisa estar no chão para pegar o raio
        self.R = self.sim.getObjectPositionBlock(self.wheelHandle[0], -1)[2]
        #self.R = 0.09749898314476013
                        
        # ROBO
        self.robotPosOrn['first']  = self.getPosOrn()
        
        # Odometry
        self.odometryPosOrn['raw']      = self.robotPosOrn['first']
        self.odometryPosOrn['compass']  = self.robotPosOrn['first']
        self.lastTime = time.time()
        self.wl = 0
        self.wr = 0
        self.encoderPos = [None] * 2
        self.encoderPos[0] = self.sim.getJointPosition(self.motorHandle[0])
        self.encoderPos[1] = self.sim.getJointPosition(self.motorHandle[1])
        
    def getJoint(self):
        return self.sim.getJointPosition(self.motorHandle[0])
        
    def getLinearVelocity(self):
        return np.linalg.norm(self.sim.getObjectVelocity(self.robotHandle)[0][:2])
        
    def move(self, left, right):
        #self.computeOdomety()
        self.wl = 2 * left
        self.wr = 2 * right
        self.sim.setJointTargetVelocity(self.motorHandle[0], left)
        self.sim.setJointTargetVelocity(self.motorHandle[1], right)
        
    def stop(self):
        self.move(0, 0)
        
    def drive(self, vLinear, vAngular):
        self.move(self.vLToDrive(vLinear,vAngular), self.vRToDrive(vLinear,vAngular)); 
        
    def vRToDrive(self, vLinear, vAngular):
        return (((2*vLinear)+(self.L*vAngular))/2*self.R);
    
    def vLToDrive(self, vLinear, vAngular):
        return (((2*vLinear)-(self.L*vAngular))/2*self.R);
        
    def getPosOrn(self):
        x, y = self.sim.getObjectPosition(self.robotHandle, -1)[:2]
        orn = self.sim.getObjectOrientation(self.robotHandle, -1)[2]
        return x, y, orn
    
    def getPosOrnOdometyRaw(self):
        return self.odometryPosOrn['raw']
    
    def getPosOrnOdometyCompass(self):
        return self.odometryPosOrn['compass']
    
    def computeOdomety(self):
        deltaTime = time.time() - self.lastTime
        self.lastTime = time.time() 
        
        deltaS = deltaTime * self.R * (self.wr + self.wl) / 2
        deltaTheta = deltaTime * self.R * (self.wr - self.wl) / self.L
        
        angle = self.odometryPosOrn['raw'][2] + (deltaTheta/2)
                
        self.odometryPosOrn['raw'] = self.odometryPosOrn['raw']  + np.array(
                [deltaS * np.cos(angle), deltaS * np.sin(angle), deltaTheta])
    
    def _computeAngularVelocityL(self):
        posF = self.sim.getJointPosition(self.motorHandle[0])
        posI = self.encoderPos[0]
        
        if (self.wl > 0) :
            if (posF<0) and (posI>0):
                posI = posI - 2 * np.pi
        else: 
            if (posF>0) and (posI<0):
                posI = posI + 2 * np.pi
               
        self.encoderPos[0] = posF
                
        return posF - posI
    
    def _computeAngularVelocityR(self):
        posF = self.sim.getJointPosition(self.motorHandle[1])
        posI = self.encoderPos[1]
        
        if (self.wl > 0) :
            if (posF<0) and (posI>0):
                posI = posI - 2 * np.pi
        else: 
            if (posF>0) and (posI<0):
                posI = posI + 2 * np.pi
               
        self.encoderPos[1] = posF
                
        return posF - posI
        
    def computeOdometyEncoder(self):
    
        wl = self._computeAngularVelocityL()*2;
        wr = self._computeAngularVelocityR()*2;
        
        print("Velocidades: {}, {}".format(wl, wr))
        
        deltaTime = time.time() - self.lastTime
        self.lastTime = time.time() 
        
        deltaS = deltaTime * self.R * (wr + wl) / 2
        deltaTheta = deltaTime * self.R * (wr - wl) / self.L
        
        angle = self.odometryPosOrn['raw'][2] + (deltaTheta/2)
                
        self.odometryPosOrn['raw'] = self.odometryPosOrn['raw']  + np.array(
                [deltaS * np.cos(angle), deltaS * np.sin(angle), deltaTheta])
        
    
    def readSonars(self):
        pointCloud = []
        distances = []
        
        for i in range(0, len(self.sonarHandle)):
            state, distance = self.sim.readProximitySensor(self.sonarHandle[i])
            
            
            
            if (state == True):
                x = self.SONAR_POSITIONS[i][0] + (distance * np.cos(self.SONAR_ANGLES[i]))
                y = self.SONAR_POSITIONS[i][1] + (distance * np.sin(self.SONAR_ANGLES[i]))
                pointCloud.append((x, y))
                distances.append(distance)
            else:
                pointCloud.append((np.inf, np.inf))
                distances.append(np.inf)
 
        return distances, pointCloud
        
    def localToGlobal(self, position):
        x, y, angle = self.getPosOrn()
        
        cos = np.cos(angle)
        sen = np.sin(angle)
        
        transformationMatrix = np.array([[cos, -sen, x], 
                                        [sen, cos, y]])
        
        return np.matmul(transformationMatrix,
                         np.concatenate((position, np.array([1]))))
    
    
    