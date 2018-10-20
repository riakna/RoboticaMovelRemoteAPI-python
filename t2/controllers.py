# -*- coding: utf-8 -*-
"""
Created on Mon Oct  8 10:27:03 2018

@author: Anderson

"""

import numpy as np
import skfuzzy as fuzz
from skfuzzy import control as ctrl

import time

class Controller:
    
    def __init__(self, reference):
        self.reference = reference
        
    def setReference(self, reference):
        self.reference = reference


class OnOffController(Controller):
    
    def __init__(self, reference, uMin, uMax):
        super().__init__(reference)
        self.uMin = uMin
        self.uMax = uMax
        
    def compute(self, measured):
        error = self.reference - measured
        if error > 0:
            return self.uMax
        
        return self.uMin
    
    
class PIDController(Controller):
    
    def __init__(self, reference, kp, ki=0, kd=0, windowSize=None):
        super().__init__(reference)
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.integral = 0
        self.lastError = None
        self.lastTime = None
        if windowSize is not None: 
            self.integralHistory = [0] * windowSize

    def compute(self, measured):

        error = self.reference - measured
        currentTime = time.time()
        
        # Propotional
        uProportional  = self.kp * error
        uIntegral = 0
        uDerivative = 0
        
        if self.lastTime is not None: 
            
            # Integral
            currentIntegral = error * (currentTime - self.lastTime)
            oldestIntegral = self.integralHistory[0]
            self.integralHistory = self.integralHistory[1:] + [None] # desloca elementos para esquerda
            self.integralHistory[-1] = currentIntegral
            self.integral += currentIntegral - oldestIntegral
            uIntegral = self.ki * self.integral
            
            # Derivative 
            currentDerivative = (error - self.lastError) / (currentTime - self.lastTime)
            uDerivative = self.kd * currentDerivative
    
        self.lastTime = currentTime
        self.lastError = error
        
        return uProportional + uIntegral + uDerivative

class OAFController:
    
    def __init__(self):
        proximityL = ctrl.Antecedent(np.arange(0, 11, 1), 'proximityL')
        proximityR = ctrl.Antecedent(np.arange(0, 11, 1), 'proximityR')

        angularVelocity = ctrl.Consequent(
                np.arange(-2, 3, 1), 'angularVelocity',  defuzzify_method='centroid')
        
        linearVelocity = ctrl.Consequent(
                np.arange(0, 6, 1), 'linearVelocity',  defuzzify_method='centroid')

        # Fuzzy Functions
        angularVelocity['right_high']    = fuzz.trapmf(angularVelocity.universe, [-2, -2, -1, 0])
        angularVelocity['zero']        = fuzz.trimf(angularVelocity.universe, [0, 0, 0])
        angularVelocity['left_high']   = fuzz.trapmf(angularVelocity.universe, [0, 1, 2, 2])

        linearVelocity['zero']   = fuzz.trimf(linearVelocity.universe, [0, 0, 0])
        linearVelocity['low']    = fuzz.trimf(linearVelocity.universe, [0, 1, 2])
        linearVelocity['medium'] = fuzz.trapmf(linearVelocity.universe, [1, 2, 5, 5])
        
        proximityL['far']           = fuzz.trapmf(proximityL.universe, [5, 6, 10, 10])
        proximityL['close']         = fuzz.trapmf(proximityL.universe, [3, 4, 5, 6])
        proximityL['very_close']    = fuzz.trapmf(proximityL.universe, [0, 0, 3, 4])
        
        proximityR['far']           = fuzz.trapmf(proximityR.universe, [5, 6, 10, 10])
        proximityR['close']         = fuzz.trapmf(proximityL.universe, [3, 4, 5, 6])
        proximityR['very_close']    = fuzz.trapmf(proximityR.universe, [0, 0, 3, 4])
        
        self.proximityL = proximityL
        self.proximityR = proximityR
        self.angularVelocity = angularVelocity
        self.linearVelocity = linearVelocity
        
        
        # Rules
        rules = []
        
        rules.append(ctrl.Rule(proximityL['very_close'] & ~proximityR['very_close'], (angularVelocity['right_high'], linearVelocity['zero'])))
        rules.append(ctrl.Rule(proximityR['very_close'] & ~proximityL['very_close'], (angularVelocity['left_high'], linearVelocity['zero'])))
        rules.append(ctrl.Rule(proximityL['close'], (angularVelocity['right_high'], linearVelocity['low'])))
        rules.append(ctrl.Rule(proximityR['close'], (angularVelocity['left_high'], linearVelocity['low'])))
        rules.append(ctrl.Rule(proximityL['far'] & proximityR['far'], (angularVelocity['zero'], linearVelocity['medium'])))
        rules.append(ctrl.Rule(proximityL['very_close'] & proximityR['very_close'], (angularVelocity['right_high'], linearVelocity['zero'])))

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

