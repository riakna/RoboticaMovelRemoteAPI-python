# -*- coding: utf-8 -*-
"""
Created on Tue Sep 18 15:32:45 2018

@author: Anderson
"""

from util import Map


mapPoints = Map()

mapPoints.loadData('pontos20l_15a')


mapPoints.saveFig("fig_odometria_simples_velocidade_baixa",  
                  ['obstaclesLaser', 'robotPathGT', 'robotPathRaw'],
                  ['black', 'red', 'blue'],
                  ['Pontos detectados pelo sensor laser', 'Caminho do robo GT', 'Caminho estimado pela odometria simples'],
                  [2, 1, 1])

mapPoints.saveFig("fig_odometria_todas_velocidade_baixa",  
                  ['obstaclesLaser', 'robotPathGT', 'robotPathRaw', 'robotPathEncoder', 'robotPathCompass'],
                  ['black', 'red', 'blue', 'green', 'orange'],
                  ['Pontos detectados pelo sensor laser', 'Caminho do robo GT', 'Caminho estimado pela odometria simples', 'Caminho estimado pela odometria encoder',  'Caminho estimado pela odometria giroscópio'],
                  [2, 1, 1, 1, 1])

mapPoints.loadData('pontos50l_30a')


mapPoints.saveFig("fig_odometria_simples_velocidade_media",  
                  ['obstaclesLaser', 'robotPathGT', 'robotPathRaw'],
                  ['black', 'red', 'blue'],
                  ['Pontos detectados pelo sensor laser', 'Caminho do robo GT', 'Caminho estimado pela odometria simples'],
                  [2, 1, 1, 1, 1])


mapPoints.saveFig("fig_odometria_encoder_compass_velocidade_media",  
                  ['obstaclesLaser', 'robotPathGT', 'robotPathEncoder', 'robotPathCompass'],
                  ['black', 'red', 'blue', 'green'],
                  ['Pontos detectados pelo sensor laser', 'Caminho do robo GT', 'Caminho estimado pela odometria encoder',  'Caminho estimado pela odometria giroscópio'],
                  [2, 1, 1, 1])

mapPoints.loadData('pontos80l_40a')

mapPoints.saveFig("fig_odometria_encoder_compass_velocidade_alta",  
                  ['obstaclesLaser', 'robotPathGT', 'robotPathEncoder', 'robotPathCompass'],
                  ['black', 'red', 'blue', 'green'],
                  ['Pontos detectados pelo sensor laser', 'Caminho do robo GT', 'Caminho estimado pela odometria encoder',  'Caminho estimado pela odometria giroscópio'],
                  [2, 1, 1, 1])


#%%
mapPoints = Map()

mapPoints.loadData('mapeamento')

mapPoints.saveFig("fig_map_laser",  
                  ['obstaclesLaser'],
                  ['blue'],
                  ['Pontos detectados pelo sensor laser'],
                  [0.001])


mapPoints.saveFig("fig_map_sonar",  
                  ['obstaclesSonar'],
                  ['red'],
                  ['Pontos detectados pelo sonar'],
                  [0.01])


mapPoints.saveFig("fig_map_laser_sonar",  
                  ['obstaclesLaser','obstaclesSonar'],
                  ['blue', 'red'],
                  ['Pontos detectados pelo sensor laser', 'Pontos detectados pelo sonar'],
                  [0.001, 0.01])

#%%
from util import Map

mapPoints = Map()
mapPoints.loadData('mapeamento')

points = mapPoints.points['obstaclesLaser']


#%%

from scipy.stats import linregress
import numpy as np
import matplotlib.pyplot as plt
from matplotlib import collections  as mc

pointsNp = np.array(points[:50])
plt.scatter(pointsNp[:,0], pointsNp[:,1])
plt.show()

#%%




segments = []

def split(points):
    
    if (len(points))<2:
        return

    pointsNp = np.array(points)
    slope, intercept, _, _, _ = linregress(pointsNp[:,0], pointsNp[:,1])
    
    distance = [np.absolute(-slope*point[0]+point[1]-intercept)/(np.sqrt(1+slope**2)) 
        for point in points]
    
    def f(x):
        return slope*x + intercept
    
    max_idx = np.argmax(distance)
    
    print(np.max(distance))
    
    if (distance[max_idx]>0.1):
        farthest_point = points[max_idx]
        
        def separator(x):
            return (-x/slope) + (farthest_point[1] + (farthest_point[0]/slope))
        
        set1 = [point for point in points if separator(point[0])>=point[1]]
        set2 = [point for point in points if separator(point[0])<=point[1]]
        
        if (len(set1)<2):
            set2 = [point for point in points if separator(point[0])<point[1]]
            split(set2) 
        elif (len(set2)<2):
            set1 = [point for point in points if separator(point[0])>point[1]]
            split(set1) 
        else:
            split(set1) 
            split(set2)   
        
        
    else:
        x_begin = np.min(pointsNp[:,0])
        x_end = np.max(pointsNp[:,0])
        segments.append([(x_begin, f(x_begin)),(x_end, f(x_end))])


split(points[:100])

print(segments)

lc = mc.LineCollection(segments, linewidths=2)
fig, ax = plt.subplots()
ax.add_collection(lc)
ax.autoscale()
ax.margins(0.1)
plt.scatter(pointsNp[:,0], pointsNp[:,1])


plt.show()

#%%

pointsNp = np.array(points)
plt.scatter(pointsNp[:,0], pointsNp[:,1])


#%%
x = np.linspace(1, 2, 1000)
plt.scatter(x, f(x))
plt.show()


# %%

from scipy.spatial import KDTree

tree = KDTree(points[:100])
neighbor_dists, neighbor_indices = tree.query(points[:100], k=4)


print(neighbor_dists)