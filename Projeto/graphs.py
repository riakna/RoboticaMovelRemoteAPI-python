# -*- coding: utf-8 -*-
"""
Created on Tue Sep 18 15:32:45 2018

@author: Anderson
"""
#%%
from util import Map

newMap = Map.loadData('dados/sub_map')

newMap.saveFig("imagens/sub_map",  
                  ['gray', 'red', 'green', 'yellow', 'black'],
                  ['Sem estratégia', 'Desvio obstáculos', 'Seguidor parede esquerda', 'Seguidor parede direita', 'Pontos detectados pelo sensor laser'],
                  [1, 10, 1, 1, 1])
#([pointsGTStNone, pointsGTSt0, pointsGTSt1, pointsGTSt2, pointsLaser])
#newMap.plotAll()
#newMap.saveData("dados/sub_map")


#%%############################
# ========= Gráficos =========# 
###############################

#%%
from util import *


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
