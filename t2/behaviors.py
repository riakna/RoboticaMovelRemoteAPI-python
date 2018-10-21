# -*- coding: utf-8 -*-
"""
Created on Sat Oct 20 18:17:23 2018

@author: Anderson
"""

from subsumption import Behavior

class AvoidObstaclesFuzzy(Behavior):
        
    def __init__(self, robot):
        self.robot = robot
    
    def check(self):
        
        state, distanceL = self.robot.sim.readProximitySensor(self.robot.sonarHandle[3])
        if (state == False):
            distanceL = 1
        state, distanceR = self.robot.sim.readProximitySensor(self.robot.sonarHandle[4])
        if (state == False):
            distanceR = 1
            
        if distanceL < 0.8 or distanceR < 0.8:
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
        
        state, distance = self.robot.sim.readProximitySensor(self.robot.sonarHandle[8])
        if (state == False):
            distance = 1
            
        if distance < 0.65:
            return True
        
        return False
        
    def action(self):
        self.robot.followWallPID(False)

    def suppress(self):
        return

    