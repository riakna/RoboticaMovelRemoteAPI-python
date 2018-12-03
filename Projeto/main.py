# -*- coding: utf-8 -*-
"""
Created on Wed Sep 12 10:44:52 2018

@author: Willian Beneducci
"""
## Extrai mapa, define o RoadMap e realiza o FollowPath
## Pre-req: executar mazeScene

from robot import Robot
import matplotlib.pyplot as plt
from imageio import imwrite
from mapping import GridMap
from probabilistic_road_map import PRM
from util import Map
from util import GraphData
from util import plot

import vrep
import time

#Create Objects
robot = Robot()
image = robot.mapImage
PRM_Object = PRM()

#FollowPath Variables
t = time.time()
Y = []
X = []
rx_real = []
ry_real = []
n = 0

#Compute Roadmap
rx,ry = PRM.compute(PRM_Object,image)

gridMap = GridMap(image)
gridMap.show()

x, y, _ = robot.getPosOrn()
x, y = gridMap.convertToMapUnit(x, y)

#Remove ocupação do robô
gridMap.removeCluster(x, y)

# FollowPath
while (time.time()-t) < 120:
        
    #laser_point_cloud = robot.readLaser()
    #laser_point_cloud = laser_point_cloud[:,:2]
    #for x in range(len(laser_point_cloud)):
        #mapPoints.addPoint('obstaclesLaser', *robot.localToGlobalGT(laser_point_cloud[x]))
    
    #Convert values
    rx_real = (rx[len(rx)-n-1]-100)/33.33-4.5
    ry_real = (ry[len(ry)-n-1]-15)/34.64-7
    
    #Show points to plot    
    output = robot.FollowPath(rx_real,ry_real)
    #print(rx_real , ry_real)
    if(output == 'true'):
        print('Arrived: ',n)
        n = n + 1

    # GT
    #x,y,orn = robot.getPosOrn()
    #X.append(x)
    #Y.append(y)  
     
    """     
    # Encoder
    
    x,y,orn = robot.getPosOrnOdometyEncoder()
    I.append(x)
    J.append(y)   
    """
        
    #Just control robot
    #robot.followWallPID(True)
    
    time.sleep(0.05)
    #robot.computeOdometryEncoder()
     
    #mapPoints.addPoint('robotPathGT', *robot.getPosOrn()[:2])
    #mapPoints.addPoint('robotPathEncoder', *robot.getPosOrnOdometyEncoder()[:2])
    
robot.stop()

        
vrep.simxFinish(-1)

#%%
## Teste potentialPlanning
## Pre-req: simpleScene

from robot import Robot
import time
import mapping as mp
from util import Map

robot = Robot(5.0)
obstacles = robot.gridMap.getObstaclesList()

mp.KP = 0.05
mp.ETA = 5 
mp.ETA_F = 1
mp.MAX = 1
mp.SHOW_GRAPHS = True
plannedPath = robot.potentialPlanningOffline(110, 80)

execPath = []

while(robot.doPlanning()):
    execPath.append((*robot.getPosOrn()[:2],))
    time.sleep(0.01)

execMap = Map([obstacles, plannedPath, execPath])

execMap.saveFig("map",  
                  ['black', 'green', 'red'],
                  ['Obstáculos', 'Caminho planejado', 'Caminho executado'],
                  [2, 3, 1],
                  'upper right')


#%%
## Teste cell decomposition
## Pre-req: mazeScene.ttt

from robot import Robot
import matplotlib.pyplot as plt
from imageio import imwrite
from mapping import GridMap
from cell_decomposition import CellDecomposition


import vrep

robot = Robot(15.0)

x, y, _ = robot.getPosOrn()

x, y = robot.gridMap.convertToMapUnit(x, y)

yd, xd = robot.gridMap.convertToMapUnit(-2.77, 5.77)

CellDecomposition(robot.gridMap, (y,x), (xd,yd), 20, 500)    
        
vrep.simxFinish(-1)


#%%
## simple_scene_potential_small_kp
## Pre-req: simpleScene

from robot import Robot
import mapping as mp

robot = Robot(5.0)
obstacles = robot.gridMap.getObstaclesList()

mp.KP = 0.01
mp.ETA = 5 
mp.ETA_F = 1
mp.MAX = 1
mp.SHOW_GRAPHS = True
plannedPath = robot.potentialPlanningOffline(110, 80)

#%%
## Teste potentialAStarPlanning 
## Pre-req: mazeScene

from robot import Robot
import time
import mapping as mp
from util import Map

robot = Robot(15.0)

mp.KP = 0.001
mp.ETA = 10 
mp.ETA_F = 1
mp.MAX = 1
mp.SHOW_GRAPHS = True

obstacles = robot.gridMap.getObstaclesList()
plannedPath = robot.potentialAStarPlanningOffline(200, 450)
execPath = []

while(robot.doPlanning()):
    execPath.append((*robot.getPosOrn()[:2],))
    time.sleep(0.01)

execMap = Map([obstacles, plannedPath, execPath])


execMap.saveFig("map",  
                  ['black', 'green', 'red'],
                  ['Obstáculos', 'Caminho planejado', 'Caminho executado'],
                  [1, 0.5, 0.5],
                  'lower right')

#%%
## Problema do minimo local
## Pre-req: mazeScene

from robot import Robot
import mapping as mp

robot = Robot(15.0)

mp.KP = 0.01
mp.ETA = 10 
mp.ETA_F = 1
mp.MAX = 1
mp.SHOW_GRAPHS = True

plannedPath = robot.potentialPlanningOffline(380, 180)


