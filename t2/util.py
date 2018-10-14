# -*- coding: utf-8 -*-
"""
Created on Thu Sep 13 19:27:01 2018

@author: Anderson
"""

import matplotlib.pyplot as plt
import matplotlib.patches as mpatches
import numpy as np
import pickle
from matplotlib.font_manager import FontProperties


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
            
    def loadData(self, name):
        with open(name+'.pkl', 'rb') as f:
            self.points = pickle.load(f)
        
    def addPoint(self, key, x, y):
        self.points[key].append((x, y))
        
    def plotAll(self):        
        
        for key, value in self.points.items():
            if (len(value)>0):
                pointsNp = np.array(value)
                plt.scatter(pointsNp[:,0], pointsNp[:,1], color=self.color[key], s=5)

        plt.show()
        
    def saveFig(self, name, keyList, color, label, s):
        
        fig, ax = plt.subplots(dpi=150)

        plt.gca().set_aspect('equal', adjustable='box')
        plt.axis('off')
        
        patches = []
    
        for i, key in enumerate(keyList):
            if (len(self.points[key])>0):
                pointsNp = np.array(self.points[key])
                ax.scatter(pointsNp[:,0], pointsNp[:,1], color=color[i], s=s[i])
                patches.append(mpatches.Patch(color=color[i], label=label[i]))
                
        ax.legend(handles=patches,loc='upper right', fontsize='x-small')

        fig.tight_layout()

        plt.savefig(name+'.png',  bbox_inches='tight')
  
    
    
    
    
    