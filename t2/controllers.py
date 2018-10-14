# -*- coding: utf-8 -*-
"""
Created on Mon Oct  8 10:27:03 2018

@author: Anderson


OAF => ObstacleAvoidanceFuzzy
"""

import numpy as np
import skfuzzy as fuzz
from skfuzzy import control as ctrl

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
        angularVelocity['right_high']   = fuzz.trapmf(angularVelocity.universe, [0, 1, 2, 2])

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
        
    
        
test = OAFController()
#test.viewGrahs()

print(test.compute(4, 2))
test.viewGrahs()

