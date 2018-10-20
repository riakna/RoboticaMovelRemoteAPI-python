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
import numpy as np

robot = Robot()
mapPoints = Map()

t = time.time()

Y = []
X = []

while (time.time()-t) < 90:
        
    laser_point_cloud = robot.readLaser()
    laser_point_cloud = laser_point_cloud[:,:2]
    for x in range(len(laser_point_cloud)):
        mapPoints.addPoint('obstaclesLaser', *robot.localToGlobalGT(laser_point_cloud[x]))
    
    output = robot.followWall(True)
    Y.append(output[0])
    X.append(time.time()-t)
    time.sleep(0.01)
    
    robot.computeOdometryEncoder()
     
    mapPoints.addPoint('robotPathGT', *robot.getPosOrn()[:2])
    mapPoints.addPoint('robotPathEncoder', *robot.getPosOrnOdometyEncoder()[:2])
    
    """
    robot.computeOdometryEncoder()
    robot.computeOdometryCompass()
    
    mapPoints.addPoint('robotPathGT', *robot.getPosOrn()[:2])
    mapPoints.addPoint('robotPathEncoder', *robot.getPosOrnOdometyEncoder()[:2])
    mapPoints.addPoint('robotPathCompass', *robot.getPosOrnOdometyCompass()[:2])
    """


"""
graphData = GraphData(X, Y)
graphData.saveData("dados/follow_wall_pd_05")



"""


mapPoints.saveData('dados/follow_wall_pid_mapa')


robot.stop()


#%%


mapPoints.plotAll()


mapPoints.loadData('dados/follow_wall_pid_mapa')

mapPoints.saveFig("map_wf_pid1",  
                  ['obstaclesLaser', 'robotPathGT', 'robotPathEncoder'],
                  ['black', 'red', 'blue'],
                  ['Pontos detectados pelo sensor laser', 'Caminho do robo GT', 'Caminho estimado pela odometria encoder'],
                  [2, 1, 1])


#%%
"""
Gráficos 
"""
#%%
d1 = GraphData.loadData("dados/follow_wall_p_05")
d2 = GraphData.loadData("dados/follow_wall_p_1")

plot([d1, d2], ['Kp = 0.5', 'Kp = 1'], 
     'Distância lida pelo sonar em função do tempo', 
     'Tempo (s)', 'Distância (m)')



#%%
d2 = GraphData.loadData("dados/follow_wall_p_1")
d3 = GraphData.loadData("dados/follow_wall_p_2")

plot([d2, d3], ['Kp = 1', 'Kp = 2'], 
     'Distância lida pelo sonar em função do tempo', 
     'Tempo (s)', 'Distância (m)')



#%%

d1 = GraphData.loadData("dados/follow_wall_pi_0005")
d2 = GraphData.loadData("dados/follow_wall_pi_005")
d3 = GraphData.loadData("dados/follow_wall_pi_001")

plot([d1, d2, d3], ['Kp = 1 | Ki = 0.005', 'Kp = 1 | Ki = 0.05', 'Kp = 1 | Ki = 0.01'], 
     'Distância lida pelo sonar em função do tempo', 
     'Tempo (s)', 'Distância (m)')

#%%

d1 = GraphData.loadData("dados/follow_wall_p_1")
d2 = GraphData.loadData("dados/follow_wall_pi_001")

plot([d1, d2], ['Kp = 1 | Ki = 0', 'Kp = 1 | Ki = 0.01'], 
     'Distância lida pelo sonar em função do tempo', 
     'Tempo (s)', 'Distância (m)')


#%%

d1 = GraphData.loadData("dados/follow_wall_pd_05")
d2 = GraphData.loadData("dados/follow_wall_pd_2")
d3 = GraphData.loadData("dados/follow_wall_pd_5")

plot([d1, d2, d3], ['Kp = 1 | Kd = 0.5', 'Kp = 1 | Kd = 2', 'Kp = 1 | Kd = 5'], 
     'Distância lida pelo sonar em função do tempo', 
     'Tempo (s)', 'Distância (m)')


