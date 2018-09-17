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

while (time.time()-t) < 100:
    
    distances, point_cloud = robot.readSonars()
        
    min_ind = np.where(distances[:8]==np.min(distances[:8]))
    min_ind = min_ind[0][0]
    
    # se o menor valor é dos sensores frontais
    if (distances[min_ind] < 0.5):
        if (0 <= min_ind < 4)  and not turning_right:
            robot.drive(0, -5*np.pi)
            turning_left = False
        elif (4 <= min_ind <= 7)  and not turning_left:
            robot.drive(0, +5*np.pi)
            turning_right = True
    else:
        robot.drive(10, 0)
        turning_left = False
        turning_right = False
                    
    for x in range(0, len(point_cloud)):
        if (point_cloud[x] != (np.inf, np.inf)):
            mapPoints.addObstacle(*robot.localToGlobal(point_cloud[x]))
    
    robot.computeOdometyEncoder()

    mapPoints.addPathGT(*robot.getPosOrn()[:2])
    mapPoints.addPathRaw(*robot.getPosOrnOdometyRaw()[:2])
    mapPoints.plot()
        
    time.sleep(0.3)
    
mapPoints.plot()
mapPoints.saveData('test')
robot.stop()