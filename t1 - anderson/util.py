# -*- coding: utf-8 -*-
"""
Created on Thu Sep 13 19:27:01 2018

@author: Anderson
"""

import matplotlib.pyplot as plt
import matplotlib.patches as mpatches
import numpy as np
import pickle


class Map:

    points = {
        'obstaclesSonar' : [],
        'obstaclesLaser' : [],
        'robotPathGT' : [],
        'robotPathRaw': [],
        'robotPathEncoder': [],
        'robotPathCompass': [],
        'robotPathEncoderCompass': []
    }
    
    color = {
        'obstaclesSonar' : 'blue',
        'obstaclesLaser' : 'hotpink',
        'robotPathGT' : 'red',
        'robotPathRaw': 'green',
        'robotPathEncoder': 'limegreen',
        'robotPathCompass': 'dodgerblue',
        'robotPathEncoderCompass': 'orange'
    }
    
    
    def saveData(self, name):
        with open(name+'.pkl', 'wb') as f: 
            pickle.dump(self.points, f)
            
            
        fig, ax = plt.subplots(figsize=(20, 20))
        plt.gca().set_aspect('equal', adjustable='box')
        


        patches = []
        for key, value in self.points.items():
            if (len(value)>0):
                pointsNp = np.array(value)
                ax.scatter(pointsNp[:,0], pointsNp[:,1], color=self.color[key], s=5)
                patches.append(mpatches.Patch(color=self.color[key], label=key))
                
        ax.legend(handles=patches,loc='upper right')
        
    
        plt.savefig('temp.png')
            
        
    def loadData(self, name):
        with open(name+'.pkl', 'rb') as f:
            self.points = pickle.load(f)
            
        
    def addPoint(self, key, x, y):
        self.points[key].append((x, y))
        
    def plotAll(self):        
        
        for key, value in self.points.items():
            _plot(value, color=self.color[key], s=1, label=key)

        plt.show()
        
    def saveFig(self, keyList):
        
        
        fig, ax = plt.subplots(figsize=(20, 20))
        plt.gca().set_aspect('equal', adjustable='box')
        
        patches = []
        for key in keyList:
            if (len(self.points[key])>0):
                pointsNp = np.array(self.points[key])
                ax.scatter(pointsNp[:,0], pointsNp[:,1], color=self.color[key], s=5)
                patches.append(mpatches.Patch(color=self.color[key], label=key))
                
        ax.legend(handles=patches,loc='upper right')
        
    
        plt.savefig('temp.png')
        

        
def _plot(points, *args, **kwargs):
    if (len(points)>0):
        pointsNp = np.array(points)
        plt.scatter(pointsNp[:,0], pointsNp[:,1],  *args, **kwargs)
        

     
    
    