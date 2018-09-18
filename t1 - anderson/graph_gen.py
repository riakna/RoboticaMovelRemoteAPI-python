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




