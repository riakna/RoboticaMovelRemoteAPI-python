# -*- coding: utf-8 -*-
"""
Created on Mon Nov 26 10:47:33 2018

@author: Anderson
"""

import numpy as np
import matplotlib.pyplot as plt
import sys

class GridMap():
    
    
    EXTEND_AREA = 15.0 #[m]
    RESOLUTION = None
    gridMap = None
    TRANSLATION = (7.5, 7.5)
    
    def __init__(self, image):
        if (image.shape[0] != image.shape[1]):
            sys.exit('Imagem para mapeamento deve ter dimensões iguais em algura e largura')
        
        self.RESOLUTION = image.shape[0]
        
        gridMap = [False if np.sum(pixel)==0 else True for pixel in image.reshape(-1, 3)]
        gridMap = np.array(gridMap)
        gridMap = gridMap.reshape(self.RESOLUTION, self.RESOLUTION)
        
        self.gridMap = gridMap
        
        
    def checkPosition(self, xReal, yReal):
        x, y = self.convertToMapUnit(xReal, yReal)
        return self.gridMap[y][x]
        
    def convertToReal(self, x, y):
        xReal = x * self.EXTEND_AREA / self.RESOLUTION - self.TRANSLATION[0]
        yReal = y * self.EXTEND_AREA / self.RESOLUTION - self.TRANSLATION[1]
        return xReal, yReal
    
    def convertToMapUnit(self, xReal, yReal):
        x = int((xReal+self.TRANSLATION[0]) * self.RESOLUTION / self.EXTEND_AREA)
        y = int((yReal+self.TRANSLATION[1]) * self.RESOLUTION / self.EXTEND_AREA)
        return x, y
    
    def removeCluster(self, x, y):
        if (self.gridMap[y][x]):
            self.gridMap[y][x] = False
            self.removeCluster(x+1, y)
            self.removeCluster(x, y+1)
            self.removeCluster(x-1, y)
            self.removeCluster(x, y-1)
            
    def show(self):
        plt.imshow(self.gridMap, origin='lower')
        
        
    """
    
    def calcPotentialField(self):
        
        pmap = np.zeros((self.RESOLUTION, self.RESOLUTION))
        
        
        
        for ix in range(self.RESOLUTION):
            x = ix * reso + minx

            for iy in range(self.RESOLUTION):
                y = iy * reso + miny
                ug = calc_attractive_potential(x, y, gx, gy)
                uo = calc_repulsive_potential(x, y, ox, oy, rr)
                uf = ug + uo
                pmap[ix][iy] = uf

"""

