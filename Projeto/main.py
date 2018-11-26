# -*- coding: utf-8 -*-
"""
Created on Wed Sep 12 10:44:52 2018

@author: Anderson
"""
## Extrai mapa e calcula o campo potencial
## Pre-req: executar mazeScene

from robot import Robot
import matplotlib.pyplot as plt
from imageio import imwrite
from mapping import GridMap


import vrep

robot = Robot()

image = robot.mapImage

plt.imshow(image)
imwrite("map.png", image)

gridMap = GridMap(image)

gridMap.show()

x, y, _ = robot.getPosOrn()

x, y = gridMap.convertToMapUnit(x, y)
gridMap.removeCluster(x, y)

        
        
vrep.simxFinish(-1)

#%%







#%% Gotogoal test
# Start Delete Here
from robot import Robot
from util import Map
#from util import GraphData
#from util import plot

import time
# End Delete Here

robot = Robot()
mapPoints = Map()
t = time.time()
Y = []
X = []

while (time.time()-t) < 60:
        
    laser_point_cloud = robot.readLaser()
    laser_point_cloud = laser_point_cloud[:,:2]
    for x in range(len(laser_point_cloud)):
        mapPoints.addPoint('obstaclesLaser', *robot.localToGlobalGT(laser_point_cloud[x]))
    
    #Show points to plot
    output = robot.GoToGoal(5,-0.85)   # Insert goal X,Y
    Y.append(output[0])
    X.append(time.time()-t)
    
    #Just control robot
    #robot.followWallPID(True)
    
    time.sleep(0.01)
    robot.computeOdometryEncoder()
     
    mapPoints.addPoint('robotPathGT', *robot.getPosOrn()[:2])
    mapPoints.addPoint('robotPathEncoder', *robot.getPosOrnOdometyEncoder()[:2])
    
    
    
#%% Obstacle fuzzy test



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
    
    output = robot.followWallPID(True)
    Y.append(output)
    X.append(time.time()-t)
    
    time.sleep(0.01)    
    robot.computeOdometryEncoder()
     
    mapPoints.addPoint('robotPathGT', *robot.getPosOrn()[:2])
    
    mapPoints.addPoint('robotPathEncoder', *robot.getPosOrnOdometyEncoder()[:2])
    
    mapPoints.addPoint('robotPathCompass', *robot.getPosOrnOdometyCompass()[:2])
    
robot.stop()

#mapPoints.saveData('follow_wall_pid_mapa.pkl')
#GraphData(X, Y).saveData('dados/follow_wall...')


#%% Subsumptions test

from robot import Robot
import time

import time

robot = Robot()
t = time.time()

pointsGTStNone = []
pointsGTSt0 = []
pointsGTSt1 = []
pointsGTSt2 = []
pointsLaser = []

while (time.time()-t) < 90:
        
    laser_point_cloud = robot.readLaser()
    laser_point_cloud = laser_point_cloud[:,:2]
    for x in range(len(laser_point_cloud)):
        pointsLaser.append((*robot.localToGlobalGT(laser_point_cloud[x]),))
        
    time.sleep(0.01)
    
    st = robot.stepSubsumptionStrategy()
    
    posGt = (*robot.getPosOrn()[:2],)

    if (st is None):
        pointsGTStNone.append(posGt)
    elif (st == 0):
        pointsGTSt0.append(posGt)
    elif (st == 1):
        pointsGTSt1.append(posGt)
    elif (st == 2):
        pointsGTSt2.append(posGt)
    
        
robot.stop()

    
#%%
#from util import NewMap

#newMap = NewMap([pointsGTStNone, pointsGTSt0, pointsGTSt1, pointsGTSt2, pointsLaser])
#newMap.plotAll()
#newMap.saveData("dados/sub_map")




