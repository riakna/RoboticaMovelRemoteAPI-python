# -*- coding: utf-8 -*-
"""
Created on Mon Oct  8 10:27:03 2018

@author: Anderson


OAF => ObstacleAvoidanceFuzzy
"""

import numpy as np
import skfuzzy as fuzz
from skfuzzy import control as ctrl
import matplotlib.pyplot as plt

class OAFController:
    
    def __init__(self):
        proximityL = ctrl.Antecedent(np.arange(0, 11, 1), 'proximityL')
        proximityR = ctrl.Antecedent(np.arange(0, 11, 1), 'proximityR')

        angularVelocity = ctrl.Consequent(
                np.arange(-2, 3, 1), 'angularVelocity',  defuzzify_method='centroid')
        
        linearVelocity = ctrl.Consequent(
                np.arange(0, 6, 1), 'linearVelocity',  defuzzify_method='centroid')

        # Fuzzy Functions
        angularVelocity['left_high']    = fuzz.trapmf(angularVelocity.universe, [-2, -2, -1, 0])
        angularVelocity['zero']        = fuzz.trimf(angularVelocity.universe, [0, 0, 0])
        angularVelocity['right_high']   = fuzz.trapmf(angularVelocity.universe, [0, 1.5, 2, 2])

        linearVelocity['zero']   = fuzz.trimf(linearVelocity.universe, [0, 0, 0])
        linearVelocity['low']    = fuzz.trimf(linearVelocity.universe, [0, 1, 2])
        linearVelocity['medium'] = fuzz.trapmf(linearVelocity.universe, [1, 2, 5, 5])
        
        proximityL['far']           = fuzz.trapmf(proximityL.universe, [4, 5, 10, 10])
        proximityL['close']         = fuzz.trimf(proximityL.universe, [3, 4, 5])
        proximityL['very_close']    = fuzz.trapmf(proximityL.universe, [0, 0, 3, 4])
        
        proximityR['far']           = fuzz.trapmf(proximityR.universe, [4, 5, 10, 10])
        proximityR['close']         = fuzz.trimf(proximityR.universe, [3, 4, 5])
        proximityR['very_close']    = fuzz.trapmf(proximityR.universe, [0, 0, 3, 4])
        
        self.proximityL = proximityL
        self.proximityR = proximityR
        self.angularVelocity = angularVelocity
        self.linearVelocity = linearVelocity
        
        
        # Rules
        rules = []
        rules.append(ctrl.Rule(proximityL['very_close'], (angularVelocity['right_high'], linearVelocity['zero'])))
        rules.append(ctrl.Rule(proximityR['very_close'], (angularVelocity['left_high'], linearVelocity['zero'])))
        rules.append(ctrl.Rule(proximityL['close'], (angularVelocity['right_high'], linearVelocity['low'])))
        rules.append(ctrl.Rule(proximityR['close'], (angularVelocity['left_high'], linearVelocity['low'])))
        rules.append(ctrl.Rule(proximityL['far'] & proximityR['far'], (angularVelocity['zero'], linearVelocity['medium'])))

        fuzzySystem = ctrl.ControlSystem(rules)
        self.fuzzySystemSim = ctrl.ControlSystemSimulation(fuzzySystem)

            
    def viewGrahs(self):
        self.proximityL.view()
        self.proximityR.view()
        self.angularVelocity.view(self.fuzzySystemSim)
        self.linearVelocity.view(self.fuzzySystemSim)
    
    def compute(self, proximityL, proximityR):
        
        self.fuzzySystemSim.input['proximityL'] = proximityL
        self.fuzzySystemSim.input['proximityR'] = proximityR
        
        self.fuzzySystemSim.compute()
                
        return self.fuzzySystemSim.output['linearVelocity'], self.fuzzySystemSim.output['angularVelocity']
        

class WFFController:
    def __init__(self):
        lateral_sensor = ctrl.Antecedent(np.arange(0, 1, .1), 'lateral_sensor')
        diagonal_sensor = ctrl.Antecedent(np.arange(0, 1, .1), 'diagonal_sensor')
        front_sensor = ctrl.Antecedent(np.arange(0, 1, .1), 'front_sensor')

        left_speed = ctrl.Consequent(np.arange(0, 8, .1), 'left_speed', defuzzify_method='centroid')
        right_speed = ctrl.Consequent(np.arange(0, 8, .1), 'right_speed', defuzzify_method='centroid')

        lateral_sensor['very_close'] = fuzz.trapmf(lateral_sensor.universe, [0, 0, 0.1, 0.3])
        lateral_sensor['close'] = fuzz.trimf(lateral_sensor.universe, [0.1, 0.3, 0.5])
        lateral_sensor['medium'] = fuzz.trimf(lateral_sensor.universe, [0.3, 0.5, 0.7])
        lateral_sensor['far'] = fuzz.trapmf(lateral_sensor.universe, [0.5, 0.7, 1, 1])
        
        diagonal_sensor['very_close'] = fuzz.trapmf(diagonal_sensor.universe, [0, 0, 0.1, 0.3])
        diagonal_sensor['close'] = fuzz.trimf(diagonal_sensor.universe, [0.1, 0.3, 0.5])
        diagonal_sensor['medium'] = fuzz.trimf(diagonal_sensor.universe, [0.3, 0.5, 0.7])
        diagonal_sensor['far'] = fuzz.trapmf(diagonal_sensor.universe, [0.5, 0.7, 1, 1])
        
        front_sensor['very_close'] =fuzz.trapmf(front_sensor.universe, [0, 0, 0.1, 0.3])
        front_sensor['close'] = fuzz.trimf(front_sensor.universe,  [0.1, 0.3, 0.5])
        front_sensor['medium'] = fuzz.trimf(front_sensor.universe, [0.3, 0.5, 0.7])
        front_sensor['far'] = fuzz.trapmf(front_sensor.universe,[0.5, 0.7, 1, 1])

        left_speed['slow'] = fuzz.trapmf(left_speed.universe, [0, 0, 1, 3])
        left_speed['medium'] = fuzz.trimf(left_speed.universe, [1, 3, 5])
        left_speed['fast'] = fuzz.trapmf(left_speed.universe, [3, 5, 8, 8])

        right_speed['slow'] = fuzz.trapmf(left_speed.universe, [0, 0, 1, 3])
        right_speed['medium'] = fuzz.trimf(left_speed.universe, [1, 3, 5])
        right_speed['fast'] = fuzz.trapmf(left_speed.universe, [3, 5, 8, 8])

        self.lateral_sensor = lateral_sensor
        self.diagonal_sensor = diagonal_sensor
        self.front_sensor = front_sensor
        self.left_speed = left_speed
        self.right_speed = right_speed

        rules = []
        
        rules.append(ctrl.Rule(front_sensor['very_close'] & lateral_sensor['very_close'] & diagonal_sensor['very_close'], (left_speed['fast'], right_speed['slow'])))
        rules.append(ctrl.Rule(front_sensor['very_close'] & lateral_sensor['very_close'] & diagonal_sensor['close'], (left_speed['fast'], right_speed['slow'])))
        rules.append(ctrl.Rule(front_sensor['very_close'] & lateral_sensor['very_close'] & diagonal_sensor['medium'], (left_speed['fast'], right_speed['slow'])))
        rules.append(ctrl.Rule(front_sensor['very_close'] & lateral_sensor['very_close'] & diagonal_sensor['far'], (left_speed['fast'], right_speed['slow'])))
        rules.append(ctrl.Rule(front_sensor['very_close'] & lateral_sensor['close'] & diagonal_sensor['very_close'], (left_speed['fast'], right_speed['slow'])))
        rules.append(ctrl.Rule(front_sensor['very_close'] & lateral_sensor['close'] & diagonal_sensor['close'], (left_speed['fast'], right_speed['slow'])))
        rules.append(ctrl.Rule(front_sensor['very_close'] & lateral_sensor['close'] & diagonal_sensor['medium'], (left_speed['fast'], right_speed['slow'])))
        rules.append(ctrl.Rule(front_sensor['very_close'] & lateral_sensor['close'] & diagonal_sensor['far'], (left_speed['fast'], right_speed['slow'])))
        rules.append(ctrl.Rule(front_sensor['very_close'] & lateral_sensor['medium'] & diagonal_sensor['very_close'], (left_speed['fast'], right_speed['slow'])))
        rules.append(ctrl.Rule(front_sensor['very_close'] & lateral_sensor['medium'] & diagonal_sensor['close'], (left_speed['fast'], right_speed['slow'])))
        rules.append(ctrl.Rule(front_sensor['very_close'] & lateral_sensor['medium'] & diagonal_sensor['medium'], (left_speed['fast'], right_speed['slow'])))
        rules.append(ctrl.Rule(front_sensor['very_close'] & lateral_sensor['medium'] & diagonal_sensor['far'], (left_speed['fast'], right_speed['slow'])))
        rules.append(ctrl.Rule(front_sensor['very_close'] & lateral_sensor['far'] & diagonal_sensor['very_close'], (left_speed['fast'], right_speed['slow'])))
        rules.append(ctrl.Rule(front_sensor['very_close'] & lateral_sensor['far'] & diagonal_sensor['close'], (left_speed['fast'], right_speed['slow'])))
        rules.append(ctrl.Rule(front_sensor['very_close'] & lateral_sensor['far'] & diagonal_sensor['medium'], (left_speed['fast'], right_speed['slow'])))
        rules.append(ctrl.Rule(front_sensor['very_close'] & lateral_sensor['far'] & diagonal_sensor['far'], (left_speed['fast'], right_speed['slow'])))

        rules.append(ctrl.Rule(front_sensor['close'] & lateral_sensor['very_close'] & diagonal_sensor['very_close'], (left_speed['fast'], right_speed['slow'])))
        rules.append(ctrl.Rule(front_sensor['close'] & lateral_sensor['very_close'] & diagonal_sensor['close'], (left_speed['medium'], right_speed['slow'])))
        rules.append(ctrl.Rule(front_sensor['close'] & lateral_sensor['very_close'] & diagonal_sensor['medium'], (left_speed['medium'], right_speed['slow'])))
        rules.append(ctrl.Rule(front_sensor['close'] & lateral_sensor['very_close'] & diagonal_sensor['far'], (left_speed['fast'], right_speed['slow'])))
        rules.append(ctrl.Rule(front_sensor['close'] & lateral_sensor['close'] & diagonal_sensor['very_close'], (left_speed['fast'], right_speed['slow'])))
        rules.append(ctrl.Rule(front_sensor['close'] & lateral_sensor['close'] & diagonal_sensor['close'], (left_speed['medium'], right_speed['slow'])))
        rules.append(ctrl.Rule(front_sensor['close'] & lateral_sensor['close'] & diagonal_sensor['medium'], (left_speed['medium'], right_speed['slow'])))
        rules.append(ctrl.Rule(front_sensor['close'] & lateral_sensor['close'] & diagonal_sensor['far'], (left_speed['fast'], right_speed['slow'])))
        rules.append(ctrl.Rule(front_sensor['close'] & lateral_sensor['medium'] & diagonal_sensor['very_close'], (left_speed['fast'], right_speed['slow'])))
        rules.append(ctrl.Rule(front_sensor['close'] & lateral_sensor['medium'] & diagonal_sensor['close'], (left_speed['medium'], right_speed['slow'])))
        rules.append(ctrl.Rule(front_sensor['close'] & lateral_sensor['medium'] & diagonal_sensor['medium'], (left_speed['medium'], right_speed['slow'])))
        rules.append(ctrl.Rule(front_sensor['close'] & lateral_sensor['medium'] & diagonal_sensor['far'], (left_speed['fast'], right_speed['slow'])))
        rules.append(ctrl.Rule(front_sensor['close'] & lateral_sensor['far'] & diagonal_sensor['very_close'], (left_speed['fast'], right_speed['slow'])))
        rules.append(ctrl.Rule(front_sensor['close'] & lateral_sensor['far'] & diagonal_sensor['close'], (left_speed['medium'], right_speed['slow'])))
        rules.append(ctrl.Rule(front_sensor['close'] & lateral_sensor['far'] & diagonal_sensor['medium'], (left_speed['medium'], right_speed['slow'])))
        rules.append(ctrl.Rule(front_sensor['close'] & lateral_sensor['far'] & diagonal_sensor['far'], (left_speed['slow'], right_speed['medium'])))
        

        rules.append(ctrl.Rule(front_sensor['medium'] & lateral_sensor['very_close'] & diagonal_sensor['very_close'], (left_speed['fast'], right_speed['slow'])))
        rules.append(ctrl.Rule(front_sensor['medium'] & lateral_sensor['very_close'] & diagonal_sensor['close'], (left_speed['medium'], right_speed['slow'])))
        rules.append(ctrl.Rule(front_sensor['medium'] & lateral_sensor['very_close'] & diagonal_sensor['medium'], (left_speed['medium'], right_speed['slow'])))
        rules.append(ctrl.Rule(front_sensor['medium'] & lateral_sensor['very_close'] & diagonal_sensor['far'], (left_speed['medium'], right_speed['slow'])))
        rules.append(ctrl.Rule(front_sensor['medium'] & lateral_sensor['close'] & diagonal_sensor['very_close'], (left_speed['fast'], right_speed['slow'])))
        rules.append(ctrl.Rule(front_sensor['medium'] & lateral_sensor['close'] & diagonal_sensor['close'], (left_speed['medium'], right_speed['slow'])))
        rules.append(ctrl.Rule(front_sensor['medium'] & lateral_sensor['close'] & diagonal_sensor['medium'], (left_speed['medium'], right_speed['slow'])))
        rules.append(ctrl.Rule(front_sensor['medium'] & lateral_sensor['close'] & diagonal_sensor['far'], (left_speed['medium'], right_speed['slow'])))
        rules.append(ctrl.Rule(front_sensor['medium'] & lateral_sensor['medium'] & diagonal_sensor['very_close'], (left_speed['fast'], right_speed['slow'])))
        rules.append(ctrl.Rule(front_sensor['medium'] & lateral_sensor['medium'] & diagonal_sensor['close'], (left_speed['medium'], right_speed['slow'])))
        rules.append(ctrl.Rule(front_sensor['medium'] & lateral_sensor['medium'] & diagonal_sensor['medium'], (left_speed['medium'], right_speed['slow'])))
        rules.append(ctrl.Rule(front_sensor['medium'] & lateral_sensor['medium'] & diagonal_sensor['far'], (left_speed['slow'], right_speed['medium'])))
        rules.append(ctrl.Rule(front_sensor['medium'] & lateral_sensor['far'] & diagonal_sensor['very_close'], (left_speed['fast'], right_speed['slow'])))
        rules.append(ctrl.Rule(front_sensor['medium'] & lateral_sensor['far'] & diagonal_sensor['close'], (left_speed['medium'], right_speed['slow'])))
        rules.append(ctrl.Rule(front_sensor['medium'] & lateral_sensor['far'] & diagonal_sensor['medium'], (left_speed['medium'], right_speed['slow'])))
        rules.append(ctrl.Rule(front_sensor['medium'] & lateral_sensor['far'] & diagonal_sensor['far'], (left_speed['slow'], right_speed['fast'])))
        

        rules.append(ctrl.Rule(front_sensor['far'] & lateral_sensor['very_close'] & diagonal_sensor['very_close'], (left_speed['fast'], right_speed['slow'])))
        rules.append(ctrl.Rule(front_sensor['far'] & lateral_sensor['very_close'] & diagonal_sensor['close'], (left_speed['medium'], right_speed['slow'])))
        rules.append(ctrl.Rule(front_sensor['far'] & lateral_sensor['very_close'] & diagonal_sensor['medium'], (left_speed['medium'], right_speed['slow'])))
        rules.append(ctrl.Rule(front_sensor['far'] & lateral_sensor['very_close'] & diagonal_sensor['far'], (left_speed['slow'], right_speed['medium'])))
        rules.append(ctrl.Rule(front_sensor['far'] & lateral_sensor['close'] & diagonal_sensor['very_close'], (left_speed['fast'], right_speed['slow'])))
        rules.append(ctrl.Rule(front_sensor['far'] & lateral_sensor['close'] & diagonal_sensor['close'], (left_speed['medium'], right_speed['slow'])))
        rules.append(ctrl.Rule(front_sensor['far'] & lateral_sensor['close'] & diagonal_sensor['medium'], (left_speed['medium'], right_speed['slow'])))
        rules.append(ctrl.Rule(front_sensor['far'] & lateral_sensor['close'] & diagonal_sensor['far'], (left_speed['medium'], right_speed['medium'])))
        rules.append(ctrl.Rule(front_sensor['far'] & lateral_sensor['medium'] & diagonal_sensor['very_close'], (left_speed['fast'], right_speed['slow'])))
        rules.append(ctrl.Rule(front_sensor['far'] & lateral_sensor['medium'] & diagonal_sensor['close'], (left_speed['slow'], right_speed['medium'])))
        rules.append(ctrl.Rule(front_sensor['far'] & lateral_sensor['medium'] & diagonal_sensor['medium'], (left_speed['slow'], right_speed['medium'])))
        rules.append(ctrl.Rule(front_sensor['far'] & lateral_sensor['medium'] & diagonal_sensor['far'], (left_speed['slow'], right_speed['medium'])))
        rules.append(ctrl.Rule(front_sensor['far'] & lateral_sensor['far'] & diagonal_sensor['very_close'], (left_speed['fast'], right_speed['slow'])))
        rules.append(ctrl.Rule(front_sensor['far'] & lateral_sensor['far'] & diagonal_sensor['close'], (left_speed['slow'], right_speed['fast'])))
        rules.append(ctrl.Rule(front_sensor['far'] & lateral_sensor['far'] & diagonal_sensor['medium'], (left_speed['slow'], right_speed['fast'])))
        rules.append(ctrl.Rule(front_sensor['far'] & lateral_sensor['far'] & diagonal_sensor['far'], (left_speed['slow'], right_speed['medium'])))
       
        fuzzySystem = ctrl.ControlSystem(rules)
        self.fuzzySystemSim = ctrl.ControlSystemSimulation(fuzzySystem)

    def viewGrahs(self):
        self.lateral_sensor.view()
        self.diagonal_sensor.view()
        self.front_sensor.view()
        self.left_speed.view(self.fuzzySystemSim)
        self.right_speed.view(self.fuzzySystemSim)
    
    def compute(self, lateral_dist, diagonal_dist, front_dist):
        
        self.fuzzySystemSim.input['lateral_sensor'] = lateral_dist
        self.fuzzySystemSim.input['diagonal_sensor'] = diagonal_dist
        self.fuzzySystemSim.input['front_sensor'] = front_dist
        
        self.fuzzySystemSim.compute()
                
        return self.fuzzySystemSim.output['left_speed'], self.fuzzySystemSim.output['right_speed']

#test = OAFController()
#test = WFFController()
#test.viewGrahs()

#print(test.compute(0.3, 0.8, np.inf))
#test.viewGrahs()
#plt.show()

