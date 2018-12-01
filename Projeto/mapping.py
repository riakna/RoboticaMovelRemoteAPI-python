# -*- coding: utf-8 -*-
"""
Created on Mon Nov 26 10:47:33 2018

@author: Anderson
"""

import numpy as np
import matplotlib.pyplot as plt
from sklearn.neighbors import KDTree
import sys
from util import plotSurface, plotContourScatter
from heapq import heappush, heappop
import itertools

# Parameters for potential field
KP = 0.001  # attractive potential gain
ETA = 1  # repulsive potential gain
ETA_F = 0.5
MAX = 0.2 # max potential 


SHOW_GRAPHS = True

class GridMap():

    gridMap = None
    RESOLUTION = None
    extendArea = None
    translation = None
    pMap =None

    def __init__(self, image, extendArea):

        if (image.shape[0] != image.shape[1]):
            sys.exit('Imagem para mapeamento deve ter dimensões iguais em algura e largura')
        
        self.RESOLUTION = image.shape[0]
        self.extendArea = extendArea
        self.translation = (extendArea/2.0, extendArea/2.0)
        
        
        gridMap = [0 if np.sum(pixel)==0 else 1 for pixel in image.reshape(-1, 3)]
        gridMap = np.array(gridMap)
        gridMap = gridMap.reshape(self.RESOLUTION, self.RESOLUTION)
        
        self.gridMap = gridMap
        
    def checkPosition(self, xReal, yReal):
        x, y = self.convertToMapUnit(xReal, yReal)
        return self.gridMap[y][x]
        
    def convertToReal(self, x, y):
        xReal = x * self.extendArea / self.RESOLUTION - self.translation[0]
        yReal = y * self.extendArea / self.RESOLUTION - self.translation[1]
        return xReal, yReal
    
    def convertToMapUnit(self, xReal, yReal):
        x = int((xReal+self.translation[0]) * self.RESOLUTION / self.extendArea)
        y = int((yReal+self.translation[1]) * self.RESOLUTION / self.extendArea)
        return x, y
    
    def removeCluster(self, x, y):
        if (self.gridMap[y][x]):
            self.gridMap[y][x] = False
            self.removeCluster(x+1, y)
            self.removeCluster(x, y+1)
            self.removeCluster(x-1, y)
            self.removeCluster(x, y-1)
            
    def getObstaclesList(self):
        
        oList = []
        
        for i in range(self.RESOLUTION):
             for j in range(self.RESOLUTION):
                 if (self.gridMap[j][i] == 1):
                     x, y = self.convertToReal(i, j)
                     oList.append((x, y))
                     
        return oList
        
            
    def show(self):
        plt.imshow(self.gridMap, origin='lower')
        
    
    ########################
    ## Planning functions ##
    ########################
        
    def calc_attractive_potential(self, x, y, gx, gy):
        return 0.5 * KP * np.hypot(x - gx, y - gy)

    def calc_repulsive_potential(self, x, y, dq):
        if (dq==0):
            return MAX
        u = 0.5 * ETA * (1.0 / dq) ** ETA_F
        if (u>MAX):
            return MAX
        return u
        
    def calcPotentialMap(self, gx, gy):
        
        # Make obstacles list
        obstacles = []
        for i in range(self.RESOLUTION):
             for j in range(self.RESOLUTION):
                 if (self.gridMap[i][j]):
                     obstacles.append([j, i])
        
        # Make kD Tree for find nearests obstacles
        kdt = KDTree(obstacles, leaf_size=30, metric='euclidean')
        
        # Make list of all points
        x = np.arange(0, self.RESOLUTION, 1)
        y = np.arange(0, self.RESOLUTION, 1)
        X, Y = np.meshgrid(x,y)
        XY = np.array([X.flatten(),Y.flatten()]).T
        
        # Find nearests and distances for all points
        distances, nearests = kdt.query(XY, k=1, return_distance=True)  
        distances = distances.reshape(self.RESOLUTION, self.RESOLUTION)
        nearests = nearests.reshape(self.RESOLUTION, self.RESOLUTION)
                
        aMap = np.zeros((self.RESOLUTION, self.RESOLUTION))
        rMap = np.zeros((self.RESOLUTION, self.RESOLUTION))

        
        for i in range(self.RESOLUTION):
             for j in range(self.RESOLUTION):
                aMap[j][i] = self.calc_attractive_potential(i, j, gx, gy)
                rMap[j][i] = self.calc_repulsive_potential(i, j, distances[j][i])
        
        self.pMap = aMap + rMap
        self.rMap = rMap
        
        if SHOW_GRAPHS:
            plotSurface(aMap, 'attractive_map.html')
            plotSurface(rMap, 'repulsive_map.html')
            plotSurface(self.pMap, 'potential_map.html')
        
                
    def calcPotentialPath(self, sx, sy):
        
        motion = [[1, 0],
              [0, 1],
              [-1, 0],
              [0, -1],
              [-1, -1],
              [-1, 1],
              [1, -1],
              [1, 1]]
        
        # Try all directions, find the minimun
        ix = sx
        iy = sy
        
        minp = float("inf")
        minix, miniy = -1, -1
        
        path = []
        
        while (True):
            for i in range(len(motion)):
                inx = int(ix + motion[i][0])
                iny = int(iy + motion[i][1])
                if iny >= len(self.pMap) or inx >= len(self.pMap[0]):
                    p = float("inf")  # outside area
                else:
                    p = self.pMap[iny][inx]
                if minp > p:
                    minp = p
                    minix = inx
                    miniy = iny
    
            # Arrived the minimum
            if minp >= self.pMap[iy][ix]:
                break
            
            path.append([minix, miniy])
            ix = minix
            iy = miniy
            
        if SHOW_GRAPHS:
            plotContourScatter(self.pMap, np.array(path), 'potential_path.html')
        
        return np.array(path)
    
    def calcPotentialAStarPath(self, sx, sy, gx, gy):
        
        
        class PriorityQueue:
    
            REMOVED = '<removed-item>'
        
            def __init__(self):
                self.pq = []
                self.entry_finder = {}             
                self.counter = itertools.count()
            
            def push(self, item, priority, size=0):
                'Add a new item or update the priority of an existing item'
                if item in self.entry_finder:
                    self.remove(item)
                count = next(self.counter)
                entry = [priority, count, size, item]
                self.entry_finder[item] = entry
                heappush(self.pq, entry)
            
            def remove(self, item):
                'Mark an existing item as REMOVED.  Raise KeyError if not found.'
                entry = self.entry_finder.pop(item)
                entry[-1] = self.REMOVED
                
            def check(self, item):
                if item in self.entry_finder:
                    return self.entry_finder[item]
                return None
            
            def pop(self):
        
                while self.pq:
                    priority, count, size, item = heappop(self.pq)
                    if item is not self.REMOVED:
                        del self.entry_finder[item]
                        return priority, item, size
                return None
    
        motion = [[1, 0],
              [0, 1],
              [-1, 0],
              [0, -1],
              [-1, -1],
              [-1, 1],
              [1, -1],
              [1, 1]]
        
        openHeap = PriorityQueue()
        closeSet = set()
        parentMatrix = np.array([[None] * self.RESOLUTION] * self.RESOLUTION)
        openHeap.push((sx, sy), 0)
        
        #for i in range(10):
        while (True):
                        
            nextNode = openHeap.pop()
            
            if nextNode is None:
                break
            
            cost = nextNode[0]
            ix = nextNode[1][0]
            iy = nextNode[1][1]
            
            
            if ix == gx and iy == gy:
                print("Find goal")
                break

            
            closeSet.add((ix, iy))
                        
            for i in range(len(motion)):
                 
                inx = int(ix + motion[i][0])
                iny = int(iy + motion[i][1])
                
                
                # If it's outside map
                if iny >= len(self.pMap) or inx >= len(self.pMap[0]):
                    continue    
                
                if iny < 0 or inx < 0:
                    continue    
                
                 # If it's a obstacle
                if self.gridMap[iny][inx] == 1:
                    continue
                
                # If it's already visited
                if (inx, iny) in closeSet:
                    continue
                
                
                incost = cost + self.rMap[iny][inx] ** 1
                
                                
                element = openHeap.check((inx, iny))
                
                if element is not None:
                    priority = element[0]
                    if (incost < priority):
                        openHeap.push((inx, iny), incost)
                        parentMatrix[iny][inx] = (ix, iy)
                else:
                    openHeap.push((inx, iny), incost)
                    parentMatrix[iny][inx] = (ix, iy)
                    
        
        ix = gx
        iy = gy
        
        path = []
        
        while parentMatrix[iy][ix] is not None:
            path.append([ix, iy])
            element = parentMatrix[iy][ix]
            ix = element[0]
            iy = element[1]
        
        if SHOW_GRAPHS:
            plotContourScatter(self.rMap, np.array(path), 'potential_astar_path.html')
        
        return np.array(path[::-1])
                
    
        

