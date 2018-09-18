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

while (time.time()-t) < 120:
        
    distances, point_cloud = robot.readSonars()

    laser_point_cloud = robot.readLaser()
        
    cont_left = 0
    cont_right = 0
    
    for i in range(1, 4):
        if distances[i] < 0.4:
            cont_left += 1
            
    for i in range(4, 7):
        if distances[i] < 0.4:
            cont_right += 1
    
    if (cont_left > 0) or (cont_right > 0):
        if (cont_left >= cont_right) or turning_left:
            robot.drive(0, -15)
            turning_left = True
        elif (cont_left < cont_right) or turning_right:
            robot.drive(0, 15)
            turning_right = True
    else:
        robot.drive(20, 0)
        turning_left = False
        turning_right = False
    

    for x in range(0, len(point_cloud)):
        if (point_cloud[x] != (np.inf, np.inf)):
            mapPoints.addPoint('obstaclesSonar', *robot.localToGlobalGT(point_cloud[x]))

    laser_point_cloud = laser_point_cloud[:,:2]
    for x in range(len(laser_point_cloud)):
        mapPoints.addPoint('obstaclesLaser', *robot.localToGlobalGT(laser_point_cloud[x]))
        
    time.sleep(0.1)
    
    robot.computeOdometry()
    robot.computeOdometryEncoder()
    robot.computeOdometryEncoderCompass()
    robot.computeOdometryCompass()
    
    mapPoints.addPoint('robotPathGT', *robot.getPosOrn()[:2])
    mapPoints.addPoint('robotPathRaw', *robot.getPosOrnOdometyRaw()[:2])
    mapPoints.addPoint('robotPathEncoder', *robot.getPosOrnOdometyEncoder()[:2])
    mapPoints.addPoint('robotPathCompass', *robot.getPosOrnOdometyCompass()[:2])
    mapPoints.addPoint('robotPathEncoderCompass', *robot.getPosOrnOdometyEncoderCompass()[:2])
    
    #mapPoints.plotAll()
    
mapPoints.plotAll()
mapPoints.saveData('test')
robot.stop()

mapPoints.saveFig("figura_odometry_raw",  
                  ['obstaclesLaser', 'robotPathGT', 'robotPathRaw'],
                  ['black', 'red', 'blue'],
                  )