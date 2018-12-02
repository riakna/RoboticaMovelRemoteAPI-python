
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


