# -*- coding: utf-8 -*-
"""
Created on Sat Oct  6 07:44:18 2018

@author: Anderson
"""

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
    

"""
variable = 10
pid_test = PIDController(20, 0.1, 0, 0, windowSize=3)

for i in range(0, 20):
    variable += pid_test.compute(variable)
    time.sleep(0.01)
    print(variable)        

"""
        
    


    