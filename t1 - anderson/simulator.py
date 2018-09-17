# -*- coding: utf-8 -*-
"""
Created on Wed Sep 12 10:08:18 2018

@author: Anderson
"""

import vrep
import sys

class Simulator:

    def __init__(self, ip, portNumber):
        self.clientId = -1;
        self.ip = ip
        self.portNumber = portNumber
        
        
    ### SIMULATION CONTROL 
    
    def connect(self):
        vrep.simxFinish(-1)
        self.clientId = vrep.simxStart(self.ip, self.portNumber, 
                                       True, True, 2000, 5)
        if self.clientId != -1:
            print ('Connected to remote API server')
            
        else:
            sys.exit('Could not connect to remote API server')
            
    def disconnect(self):
        if self.clientId != -1:
            vrep.simxFinish(self.clientId)
            
    def pause(self):
        if self.clientId != -1:
            vrep.simxPauseCommunication(self.clientId, 0)
            
    def resume(self):
        if self.clientId != -1:
            vrep.simxPauseCommunication(self.clientId, 1)
    
     ### INTERFACE
     
    def getHandle(self, name):
        returnCode, handle = vrep.simxGetObjectHandle(
                self.clientId, name, vrep.simx_opmode_oneshot_wait)
        if (returnCode != vrep.simx_return_ok) :
            sys.exit("Unable to receive handle")
            
        return handle;
    
    def getPingTime(self, name):
        returnCode, pingTime = vrep.simxGetPingTime(self.clientId)
        if (returnCode != vrep.simx_return_ok) :
            sys.exit("Unable to get ping time")
            
        return pingTime;
    
    def getLastCmdTime(self, name):
        returnCode, cmdTime = vrep.getLastCmdTime(self.clientId)
        if (returnCode != vrep.simx_return_ok) :
            sys.exit("Unable to get cmd time")
            
        return cmdTime;
    
    
    
    ### ACTUATORS
    def setJointTargetVelocity(self, handle, velocity):
        returnCode = vrep.simxSetJointTargetVelocity(self.clientId, handle, velocity, vrep.simx_opmode_streaming)
        
        if (returnCode != vrep.simx_return_ok) and (returnCode != vrep.simx_return_novalue_flag):
            sys.exit("Unable to set target velocity")
    
    def initJointPositionStream(self, handle):
        initStream(vrep.simxGetJointPosition, self.clientId, handle)       
    def getJointPosition(self, handle):
        return getBuffer(vrep.simxGetJointPosition, self.clientId, handle)    
         
    def initObjectPositionStream(self, handle, relativeHandle):
        initStream(vrep.simxGetObjectPosition, self.clientId, handle, relativeHandle)       
    def getObjectPosition(self, handle, relativeHandle):
        return getBuffer(vrep.simxGetObjectPosition, self.clientId, handle, relativeHandle)
    def getObjectPositionBlock(self, handle, relativeHandle):
        returnCode, position = vrep.simxGetObjectPosition(
                self.clientId, handle, relativeHandle, vrep.simx_opmode_blocking)
        if (returnCode != vrep.simx_return_ok):
            sys.exit("Unable to get position in blocking mode")
        return position
    
    def initObjectOrientationStream(self, handle, relativeHandle):
        initStream(vrep.simxGetObjectOrientation, self.clientId, handle, relativeHandle)      
    def getObjectOrientation(self, handle, relativeHandle):
        return getBuffer(vrep.simxGetObjectOrientation, self.clientId, handle, relativeHandle)

    def initObjectVelocity(self, handle):
        initStream(vrep.simxGetObjectVelocity, self.clientId, handle)  
    def getObjectVelocity(self, handle):
        return getBuffer(vrep.simxGetObjectVelocity, self.clientId, handle)
    
    def initProximitySensor(self, sensorHandle):
        initStream(vrep.simxReadProximitySensor, self.clientId, sensorHandle) 
    def readProximitySensor(self, sensorHandle):
        detectionState, detectedPoint, detectedObjectHandle, detectedSurfaceNormalVector = getBuffer(
                vrep.simxReadProximitySensor, self.clientId, sensorHandle)
        return detectionState, detectedPoint[2]
    
    def initVisionSensorImage(self, sensorHandle):
        initStream(vrep.simxGetVisionSensorImage, self.clientId, sensorHandle, 0) 
    def getVisionSensorImage(self, sensorHandle):
        return getBuffer(vrep.simxGetVisionSensorImage, self.clientId, sensorHandle, 0)
        
    def initFloatSignal(self, signalName):
        initStream(vrep.simxGetFloatSignal, self.clientId, signalName) 
    def getFloatSignal(self, signalName):
        return getBuffer(vrep.simxGetFloatSignal, self.clientId, signalName)
    
    
    
## Decorator functions
        
def initStream(func, *args, **kwargs):
    returnCode, *_ = func(*args, **kwargs, operationMode=vrep.simx_opmode_streaming)
        
    if (returnCode != vrep.simx_return_ok) and (returnCode != vrep.simx_return_novalue_flag):
        sys.exit("Error on initStream "+func.__name__)   
        
        
def getBuffer(func, *args, **kwargs):
    
    while (True):
        returnCode, *returnData = func(*args, **kwargs, operationMode=vrep.simx_opmode_buffer)
        if (returnCode==vrep.simx_return_ok):
            break
        
        print(returnCode)
        
    if (len(returnData)==1):
       return returnData[0] 
    
    return returnData
    
    
    
    
    