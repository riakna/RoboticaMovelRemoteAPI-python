# -*- coding: utf-8 -*-
"""
Created on Thu Sep 13 19:27:01 2018

@author: Anderson
"""

import matplotlib.pyplot as plt
import matplotlib.patches as mpatches
import numpy as np
import pickle
import plotly.offline as py
import plotly.graph_objs as go

class Savable:
    
    def saveData(self, name):
        with open(name+'.pkl', 'wb') as f: 
            pickle.dump(self, f)
            
    @classmethod
    def loadData(clas, name):
        with open(name+'.pkl', 'rb') as f:
            return pickle.load(f)

class GraphData(Savable):
    
    def __init__(self, X, Y):
        self.X = X
        self.Y = Y
        
    
def plotSurface(grid, name='elevations-3d-surface.html'):
    data = [go.Surface(z=grid)]
    
    layout = go.Layout(
        autosize=True,
    )
    
    fig = go.Figure(data=data, layout=layout)
    py.plot(fig, filename=name, auto_open=True)
    
    
def plotContourScatter(grid, path, name='contour-scatter.html'):

    trace1 = go.Contour(
        z=grid,
        ncontours=30,
        showscale=True
    )
    
    trace2 = go.Scatter(
        x=path[:,0],
        y=path[:,1],
        mode='markers+lines',
        name='steepest',
        line=dict(
            color='black'
        )
    )
    
    data = [trace1, trace2]
    
    layout = go.Layout(
        autosize=True,
        yaxis=dict(scaleanchor="x", scaleratio=1)
    )
    
    fig = go.Figure(data=data, layout=layout)
    
    py.plot(fig, filename=name, auto_open=True)
        
    
def plot(graphDataList, legendList=[], title="", xLabel="", yLabel = ""):

    plt.title(title)

    for data in graphDataList:
        lines = plt.plot(data.X, data.Y)
        
    plt.setp(lines, linewidth=2.0)
    plt.ylabel(yLabel)
    plt.xlabel(xLabel)
    
    plt.legend(tuple(legendList), loc='upper right')
    plt.show()
    
class Map(Savable):
    
    def __init__(self, data):
        self.data = data
        
    def plotAll(self):        
        
        for value in self.data:
            if (len(value)>0):
                pointsNp = np.array(value)
                plt.scatter(pointsNp[:,0], pointsNp[:,1], s=5)

        plt.show()
        
    def saveFig(self, name,  color, label, s, loc='upper right'):
        
        fig, ax = plt.subplots(dpi=150)

        plt.gca().set_aspect('equal', adjustable='box')
        plt.axis('off')
        
        patches = []
    
        for i, value in enumerate(self.data):
            if (len(value)>0):
                pointsNp = np.array(value)
                ax.scatter(pointsNp[:,0], pointsNp[:,1], color=color[i], s=s[i])
                patches.append(mpatches.Patch(color=color[i], label=label[i]))
                
        ax.legend(handles=patches,loc=loc, fontsize='x-small')

        fig.tight_layout()

        plt.savefig(name+'.png',  bbox_inches='tight')
    
    
    