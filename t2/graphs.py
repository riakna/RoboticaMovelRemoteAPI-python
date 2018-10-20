# -*- coding: utf-8 -*-
"""
Created on Tue Sep 18 15:32:45 2018

@author: Anderson
"""
from util import Map
from util import GraphData
from util import plot

#%%############################
# ========= Mapas ============# 
###############################

#%%

mapPoints = Map.loadData('dados/obstacle_avoid_fuzzy_mapa')

mapPoints.saveFig("imagens/map_oa_fuzzy1",  
                  ['obstaclesLaser', 'robotPathGT', 'robotPathEncoder'],
                  ['black', 'red', 'blue'],
                  ['Pontos detectados pelo sensor laser', 'Caminho do robo GT', 'Caminho estimado pela odometria encoder'],
                  [2, 1, 1])


#%%

mapPoints = Map.loadData('dados/follow_wall_pid_mapa')

mapPoints.saveFig("imagens/map_wf_pid1",  
                  ['obstaclesLaser', 'robotPathGT', 'robotPathEncoder'],
                  ['black', 'red', 'blue'],
                  ['Pontos detectados pelo sensor laser', 'Caminho do robo GT', 'Caminho estimado pela odometria encoder'],
                  [2, 1, 1])


#%%############################
# ========= Gráficos =========# 
###############################

#%%
d1 = GraphData.loadData("dados/follow_wall_p_05")
d2 = GraphData.loadData("dados/follow_wall_p_1")
d3 = GraphData.loadData("dados/follow_wall_p_2")

plot([d1, d2, d3], ['Kp = 0.5', 'Kp = 1', 'Kp = 2'], 
     '', 
     'Tempo (s)', 'Distância (m)')


#%%

d1 = GraphData.loadData("dados/follow_wall_pi_0005")
d2 = GraphData.loadData("dados/follow_wall_pi_005")
d3 = GraphData.loadData("dados/follow_wall_pi_001")

plot([d1, d2, d3], ['Kp = 1 | Ki = 0.005', 'Kp = 1 | Ki = 0.05', 'Kp = 1 | Ki = 0.01'], 
     '', 
     'Tempo (s)', 'Distância (m)')

#%%

d1 = GraphData.loadData("dados/follow_wall_pd_05")
d2 = GraphData.loadData("dados/follow_wall_pd_2")
d3 = GraphData.loadData("dados/follow_wall_pd_5")

plot([d1, d2, d3], ['Kp = 1 | Kd = 0.5', 'Kp = 1 | Kd = 2', 'Kp = 1 | Kd = 5'], 
     '', 
     'Tempo (s)', 'Distância (m)')
