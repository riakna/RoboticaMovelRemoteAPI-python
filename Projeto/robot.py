# -*- coding: utf-8 -*-
"""
Created on Wed Sep 12 10:34:58 2018

@author: Anderson
"""

import numpy as np
from simulator import Simulator
import time
from mapping import GridMap
from controllers import PIDController
import math

class Robot:

    # Handles 
    robotHandle = None
    motorHandle = [None] * 2
    wheelHandle = [None] * 2
    sonarHandle = [None] * 16    
    
    # Constants
    SONAR_POSITIONS = [None] * 16   
    PI = np.pi
    SONAR_ANGLES = np.array([PI/2, 50/180.0*PI, 30/180.0*PI, 10/180.0*PI, 
                       -10/180.0*PI, -30/180.0*PI, -50/180.0*PI, -PI/2, -PI/2, 
                       -130/180.0*PI, -150/180.0*PI, -170/180.0*PI, 170/180.0*PI,
                       150/180.0*PI, 130/180.0*PI, PI/2]) 
    
    L = None
    L2 = None
    R = None
    
    # Map Ground truth
    gridMap = None
    
    def __init__(self, extendArea):

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
        #self.visionHandle = self.sim.getHandle('camera')
        self.visionHandle = 0
            
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
        #self.sim.initVisionSensorImage(self.visionHandle)
        self.sim.initObjectVelocity(self.robotHandle)
        self.sim.initJointPositionStream(self.motorHandle[0])
        self.sim.initJointPositionStream(self.motorHandle[1])
        self.sim.initLaserSensor("measuredDataAtThisTime")
        

        ### -----------
        ### POSTIONS    
        
        # SONAR (em relacao ao frame do robo)
        for x in range(0, len(self.sonarHandle)):
            self.SONAR_POSITIONS[x] = self.sim.getObjectPositionBlock(
                    self.sonarHandle[x], self.robotHandle) 
            
        self.L = np.linalg.norm(self.sim.getObjectPositionBlock(
                self.wheelHandle[0], self.wheelHandle[1]))
        
        
        self.L2 = -self.sim.getObjectPositionBlock(self.wheelHandle[0], self.robotHandle)[0]
                
        # Robo precisa estar no chão para pegar o raio
        self.R = self.sim.getObjectPositionBlock(self.wheelHandle[0], -1)[2]
        #self.R = 0.09749898314476013
                        
        # ROBO
        firstRobotPosOrn = self.getPosOrn()
        
        # Odometry
        self.odometryPosOrn = {}
        self.odometryPosOrn['raw']              = firstRobotPosOrn
        self.odometryPosOrn['encoder']          = firstRobotPosOrn
        self.odometryPosOrn['compass']          = firstRobotPosOrn
        
        self.odometryEncoder = {}
        self.odometryEncoder['encoder'] = [None] * 2
        self.odometryEncoder['encoder'][0] = self.sim.getJointPosition(self.motorHandle[0])
        self.odometryEncoder['encoder'][1] = self.sim.getJointPosition(self.motorHandle[1])
        self.odometryEncoder['compass'] = [None] * 2
        self.odometryEncoder['compass'][0] = self.sim.getJointPosition(self.motorHandle[0])
        self.odometryEncoder['compass'][1] = self.sim.getJointPosition(self.motorHandle[1])
        
        self.odometryTimeRaw = time.time()
        self.odometryTimeCompass = time.time()
        self.odometryTimeEncoderCompass = time.time()
 
        self.motorW = [None] * 2
        self.motorW[0] = 0
        self.motorW[1] = 0
        
        
        # Map grid
        x, y, _ = self.getPosOrn()
        mapImage = self.sim.getVisionSensorImageBlock(self.sim.getHandle('mapSensor'))
        self.gridMap = GridMap(mapImage, extendArea)
        x, y, _ = self.getPosOrn()
        x, y = self.gridMap.convertToMapUnit(x, y)
        self.gridMap.removeCluster(x, y)
        self.gridMap.show()
        
    
        # Controllers
        self.goToGoalPID = PIDController(0, 50, 0, 15, windowSize=1000)
        self.anglePID = PIDController(0, 10, 0, 10, windowSize=1000)   


    ### -----------
    ### DRIVE  
    
    def move(self, left, right):
        self.computeOdometry()
        self.motorW[0] = left
        self.motorW[1] = right
        self.sim.setJointTargetVelocity(self.motorHandle[0], left)
        self.sim.setJointTargetVelocity(self.motorHandle[1], right)
        
    def stop(self):
        self.move(0, 0)
        
    def drive(self, vLinear, vAngular):
        self.move(self.vLToDrive(vLinear,vAngular), self.vRToDrive(vLinear,vAngular)) 
        
    def vRToDrive(self, vLinear, vAngular):
        return (((2*vLinear)+(self.L*vAngular))/2*self.R)
    
    def vLToDrive(self, vLinear, vAngular):
        return (((2*vLinear)-(self.L*vAngular))/2*self.R)
    
    ### -----------
    ### SONAR  

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
    
    
    ### -----------
    ### LASER

    def readLaser(self):
        pointCloud = self.sim.readLaserSensor("measuredDataAtThisTime")
        laser_array = np.reshape(pointCloud, (int(len(pointCloud)/3),3))

        return laser_array

    ### -----------
    ### IMAGE
    def getSensorViewImage(self):
        resolution, image_array = self.sim.getVisionSensorImage(self.visionHandle)
        image = np.array(image_array, dtype=np.uint8)#Create a numpy array with uint8 type
        image.resize([resolution[1], resolution[0],3])

        return image


    ### -----------
    ### ODOMETRY  
    
    def _computeMotorAngularDisplacement(self, i, name):
        posF = self.sim.getJointPosition(self.motorHandle[i])
        posI = self.odometryEncoder[name][i]
        self.odometryEncoder[name][i] = posF
        
        diff = posF - posI
        
        if (diff >= np.pi):
            diff = 2*np.pi - diff
        elif (diff <= -np.pi):
            diff = 2*np.pi + diff
        
        return diff
    
    def _computeOdometry(self, posOrn, thetaL, thetaR, deltaTheta):
        deltaS = self.R * (thetaR + thetaL) / 2
        
        if deltaTheta is None:
            deltaTheta = self.R * (thetaR - thetaL) / self.L
        
        angle = posOrn[2] + (deltaTheta/2)
        
        return posOrn + np.array(
                [deltaS * np.cos(angle) - deltaTheta* self.L2 * np.sin(angle), 
                 deltaS * np.sin(angle) + deltaTheta* self.L2 * np.cos(angle), 
                 deltaTheta])
        
    def computeOdometry(self):
        
        deltaTime = time.time() - self.odometryTimeRaw
        self.odometryTimeRaw = time.time() 
        
        thetaL = self.motorW[0] * deltaTime
        thetaR = self.motorW[1] * deltaTime
        
        self.odometryPosOrn['raw'] = self._computeOdometry(
                self.odometryPosOrn['raw'], thetaL, thetaR, None)
        
    def computeOdometryEncoder(self):
        thetaL = self._computeMotorAngularDisplacement(0, 'encoder')
        thetaR = self._computeMotorAngularDisplacement(1, 'encoder')

        self.odometryPosOrn['encoder'] = self._computeOdometry(
                self.odometryPosOrn['encoder'], thetaL, thetaR, None)
         
    def computeOdometryCompass(self):
        
        deltaTime = time.time() - self.odometryTimeCompass
        self.odometryTimeCompass = time.time() 
        
        thetaL = self._computeMotorAngularDisplacement(0, 'compass')
        thetaR = self._computeMotorAngularDisplacement(1, 'compass')
        deltaTheta = self.sim.getFloatSignal("gyroZ") * deltaTime
        
        self.odometryPosOrn['compass'] = self._computeOdometry(
                self.odometryPosOrn['compass'], thetaL, thetaR, deltaTheta)
        
    ### -----------
    ### GETTERS  
    
    def localToGlobalGT(self, position):
        return localToGlobal(self.getPosOrn(), position)
    
    def localToGlobalOdometry(self, position):
        return localToGlobal(self.getPosOrn(), position)
    
    def getPosOrn(self):
        x, y = self.sim.getObjectPosition(self.robotHandle, -1)[:2]
        orn = self.sim.getObjectOrientation(self.robotHandle, -1)[2]
        return x, y, orn
    
    def getPosOrnOdometyRaw(self):
        return self.odometryPosOrn['raw']
    
    def getPosOrnOdometyCompass(self):
        return self.odometryPosOrn['compass']
    
    def getPosOrnOdometyEncoder(self):
        return self.odometryPosOrn['encoder']
    
    def getPosOrnOdometyEncoderCompass(self):
        return self.odometryPosOrn['encoder_compass']
    
    def getLinearVelocity(self):
        return np.linalg.norm(self.sim.getObjectVelocity(self.robotHandle)[1][2])
    
    ### -----------------
    ### Path Planning

    def potentialPlanningOffline(self, gx, gy):
        self.gridMap.calcPotentialMap(gx, gy)
        x, y, _ = self.getPosOrn()
        x, y = self.gridMap.convertToMapUnit(x, y)
        self.pathPlanning = self.gridMap.calcPotentialPath(x, y)
        return [self.gridMap.convertToReal(*pos) for pos in self.pathPlanning]
        
    def potentialAStarPlanningOffline(self, gx, gy):
        self.gridMap.calcPotentialMap(gx, gy)
        x, y, _ = self.getPosOrn()
        x, y = self.gridMap.convertToMapUnit(x, y)
        self.pathPlanning = self.gridMap.calcPotentialAStarPath(x, y, gx, gy) 
    
        return [self.gridMap.convertToReal(*pos) for pos in self.pathPlanning]

    def doPlanning(self):
        if (len(self.pathPlanning)):
            nextStep = self.pathPlanning[0]
            x, y = self.gridMap.convertToReal(*nextStep)
            distance = self.goToGoal(x, y)
            if (distance<0.2):
                self.pathPlanning = self.pathPlanning[1:]
            return True
        else:
            self.stop()
            return False
        
        
        
    
    ### -----------------
    ### BEHAVIORS ACTIONS
    def goToGoal(self,x_final,y_final):
        
        # Get actual position
        x, y, orn = self.getPosOrn()
        
        # Compute angle to objective
        orn_final = math.atan2((y_final-y),(x_final-x))

        # PID control to adjust velocity to final objective
        distance = math.sqrt((x_final-x)**2+(y_final-y)**2)
        linearVelocity = -self.goToGoalPID.compute(distance)
        
        
        # PID control to adjust angular velocity to correct angle
        angle = orn - orn_final
        
        if angle > np.pi:
            angle = angle - 2 * np.pi
        elif angle < -np.pi:
            angle = angle + 2 * np.pi
            
        angleVelocity = self.anglePID.compute(angle)
        
        
                
        # Limit PID
        if (linearVelocity > 10):
            linearVelocity = 10
                                
        # Linear velocity should be smaller if it's not in correct direction
        linearVelocity = (np.cos(angle)**15) * linearVelocity
        
        if (linearVelocity < 0):
            linearVelocity = 0
        
        # Limit PID
        if (angleVelocity > 10):
            angleVelocity = 10
        elif (angleVelocity < -10):
            angleVelocity = -10      
            
        
        self.drive(5 * linearVelocity, 10 * angleVelocity)  
    
        return distance            
    

        
def localToGlobal(posOrnSource, pos):
    x, y, angle = posOrnSource
    
    
    cos = np.cos(angle)
    sen = np.sin(angle)
    
    transformationMatrix = np.array([[cos, -sen, x], 
                                    [sen, cos, y]])
    
    return np.matmul(transformationMatrix,
                     np.concatenate((pos, np.array([1]))))
    