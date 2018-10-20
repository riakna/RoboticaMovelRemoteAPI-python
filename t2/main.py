# -*- coding: utf-8 -*-
"""
Created on Wed Sep 12 10:44:52 2018

@author: Anderson
"""

from robot import Robot
from util import Map
from util import GraphData
from util import plot

import time


#%% Obstacle fuzzy test

robot = Robot()
mapPoints = Map()
t = time.time()

while (time.time()-t) < 40:
        
    laser_point_cloud = robot.readLaser()
    laser_point_cloud = laser_point_cloud[:,:2]
    for x in range(len(laser_point_cloud)):
        mapPoints.addPoint('obstaclesLaser', *robot.localToGlobalGT(laser_point_cloud[x]))
        
    time.sleep(0.01)
    robot.avoidObstacles()
    
    robot.computeOdometryEncoder()
     
    mapPoints.addPoint('robotPathGT', *robot.getPosOrn()[:2])
    mapPoints.addPoint('robotPathEncoder', *robot.getPosOrnOdometyEncoder()[:2])

robot.stop()

#mapPoints.saveData('dados/obstacle_avoid_fuzzy_mapa')


#%% Wall Follow PID test

robot = Robot()
mapPoints = Map()
t = time.time()
Y = []
X = []

while (time.time()-t) < 40:
        
    laser_point_cloud = robot.readLaser()
    laser_point_cloud = laser_point_cloud[:,:2]
    for x in range(len(laser_point_cloud)):
        mapPoints.addPoint('obstaclesLaser', *robot.localToGlobalGT(laser_point_cloud[x]))
    
    output = robot.followWall(True)
    Y.append(output[0])
    X.append(time.time()-t)
    
    time.sleep(0.01)
    robot.followWall()
    
    robot.computeOdometryEncoder()
     
    mapPoints.addPoint('robotPathGT', *robot.getPosOrn()[:2])
    mapPoints.addPoint('robotPathEncoder', *robot.getPosOrnOdometyEncoder()[:2])
    
robot.stop()

#mapPoints.saveData('follow_wall_pid_mapa.pkl')
#GraphData(X, Y).saveData('dados/follow_wall...')




