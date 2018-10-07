# -*- coding: utf-8 -*-
"""
Created on Mon Sep 10 12:42:53 2018

@author: Anderson
"""

errorCode = vrep.simxSetJointTargetVelocity(clientID, left_motor_handle, 2, vrep.simx_opmode_streaming)
errorCode = vrep.simxSetJointTargetVelocity(clientID, right_motor_handle, 2, vrep.simx_opmode_streaming)
