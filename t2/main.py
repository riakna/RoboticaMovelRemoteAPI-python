# -*- coding: utf-8 -*-
"""
Created on Wed Sep 12 10:44:52 2018

@author: Anderson
"""

from robot import Robot
from util import Map
import controllers
import time
import numpy as np
import matplotlib as plt

robot = Robot()
mapPoints = Map()

t = time.time()

turning_left = False
turning_right = False
controller = controllers.WFFController()
#controller.viewGrahs()
#plt.pyplot.show()
while (time.time()-t) < 200:
    """#teste wff controller   
    distances, point_cloud = robot.readSonars()
    left, right = controller.compute(distances[15], distances[2], distances[3])
    robot.move(left, right)
    """
    
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
    
    
    #robot.followWall(True)
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
