# -*- coding: utf-8 -*-
"""
Created on Thu Sep 13 19:27:01 2018

@author: Anderson
"""

import matplotlib.pyplot as plt
import numpy as np
import pickle


class Map:
    
    obstacles = []
    robotPathGT = []
    robotPathRaw = []
    robotPathCompass = []
    
    def saveData(self, name):
        with open(name+'.pkl', 'wb') as f: 
            pickle.dump([self.obstacles, self.robotPathGT, 
                         self.robotPathRaw, self.robotPathCompass], f)
        
    def loadData(self, name):
        with open(name+'.pkl', 'rb') as f:
            self.obstacles, self.robotPathGT, self.robotPathRaw, self.robotPathCompass = pickle.load(f)
    
    def addObstacle(self, x, y):
        self.obstacles.append((x, y))
    
    def addPathGT(self, x, y):
        self.robotPathGT.append((x, y))
    
    def addPathCompass(self, x, y):
        self.robotPathCompass.append((x, y))
        
    def addPathRaw(self, x, y):
        self.robotPathRaw.append((x, y))
        
    def plot(self):
        _plot(self.obstacles, color='blue', s=3)
        _plot(self.robotPathGT, color='red', s=2)
        _plot(self.robotPathCompass, color='pink', s=3)
        _plot(self.robotPathRaw, color='green', s=2)

        plt.show()
        
def _plot(points, *args, **kwargs):
    if (len(points)>0):
        pointsNp = np.array(points)
        plt.scatter(pointsNp[:,0], pointsNp[:,1],  *args, **kwargs)