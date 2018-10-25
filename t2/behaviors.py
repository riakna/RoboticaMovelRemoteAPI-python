# -*- coding: utf-8 -*-
"""
Created on Sat Oct 20 18:17:23 2018

@author: Anderson
"""

from subsumption import Behavior
import numpy as np

class AvoidObstaclesFuzzy(Behavior):
        
    def __init__(self, robot):
        self.robot = robot
    
    def check(self):
        
        distancesL = []
        
        for i in range(2, 4):
            state, distance = self.robot.sim.readProximitySensor(self.robot.sonarHandle[i])
            if (state == True):
                distancesL.append(np.abs(distance*np.cos(self.robot.SONAR_ANGLES[i])))
            else:
                distancesL.append(1)
                
        distancesR = []
        
        for i in range(4, 6):
            state, distance = self.robot.sim.readProximitySensor(self.robot.sonarHandle[i])
            if (state == True):
                distancesR.append(np.abs(distance*np.cos(self.robot.SONAR_ANGLES[i])))
            else:
                distancesR.append(1)
    
        if np.min(distancesL) < 0.35 or np.min(distancesR) < 0.35:
            return True
        
        return False
        
    def action(self):
        self.robot.avoidObstaclesFuzzy()

    def suppress(self):
        return

class FollowLeftWallPID(Behavior):
    
    def __init__(self, robot):
        self.robot = robot
    
    def check(self):
        
        state, distance = self.robot.sim.readProximitySensor(self.robot.sonarHandle[0])
        if (state == False):
            distance = 1
            
        if distance < 0.65:
            return True
        
        return False
        
    def action(self):
        self.robot.followWallPID(True)

    def suppress(self):
        return 
        
class FollowRightWallPID(Behavior):
    
    def __init__(self, robot):
        self.robot = robot
    
    def check(self):
        
        state, distance = self.robot.sim.readProximitySensor(self.robot.sonarHandle[7])
        if (state == False):
            distance = 1
            
        if distance < 0.65:
            return True
        
        return False
        
    def action(self):
        self.robot.followWallPID(False)

    def suppress(self):
        return

    