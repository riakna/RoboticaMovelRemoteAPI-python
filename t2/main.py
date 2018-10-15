# -*- coding: utf-8 -*-
"""
Created on Wed Sep 12 10:44:52 2018

@author: Anderson
"""

from robot import Robot
from util import Map
import time
import numpy as np

robot = Robot()
mapPoints = Map()

t = time.time()

turning_left = False
turning_right = False

while (time.time()-t) < 2000:
        

    """
    distances, point_cloud = robot.readSonars()
    laser_point_cloud = robot.readLaser()
    laser_point_cloud = laser_point_cloud[:,:2]
    for x in range(len(laser_point_cloud)):
        mapPoints.addPoint('obstaclesLaser', *robot.localToGlobalGT(laser_point_cloud[x]))
        
    for x in range(0, len(point_cloud)):
        if (point_cloud[x] != (np.inf, np.inf)):
            mapPoints.addPoint('obstaclesSonar', *robot.localToGlobalGT(point_cloud[x]))

    """
    
    robot.avoidObstacles()
    time.sleep(0.1)
    
    robot.computeOdometryEncoder()
    robot.computeOdometryCompass()
    
    mapPoints.addPoint('robotPathGT', *robot.getPosOrn()[:2])
    mapPoints.addPoint('robotPathEncoder', *robot.getPosOrnOdometyEncoder()[:2])
    mapPoints.addPoint('robotPathCompass', *robot.getPosOrnOdometyCompass()[:2])
    
    #mapPoints.plotAll()
    
mapPoints.plotAll()
mapPoints.saveData('test')
robot.stop()
