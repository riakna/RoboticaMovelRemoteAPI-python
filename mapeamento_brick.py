# -*- coding: utf-8 -*-
"""
Created on Sun Sep  2 10:04:06 2018

@author: Anderson
"""
import vrep
import sys
import time
import numpy as np
import math
import matplotlib.pyplot as plt

import _thread

def input_thread(a_list):
    input()
    a_list.append(True)

PI = math.pi

# Conexao com V-REP
vrep.simxFinish(-1)
clientID = vrep.simxStart('127.0.0.1', 25100, True, True, 2000, 5)

if clientID != -1:
    print('Connected to remote API server')

else:
    print('Connection not successful')
    sys.exit('Could not connect')

# Handle do robo
errorCode, robot_handle = vrep.simxGetObjectHandle(clientID,
                                                   'Pioneer_p3dx', vrep.simx_opmode_oneshot_wait)

# Pega os handles dos motores
errorCode, left_motor_handle = vrep.simxGetObjectHandle(clientID,
                                                        'Pioneer_p3dx_leftMotor', vrep.simx_opmode_oneshot_wait)
errorCode, right_motor_handle = vrep.simxGetObjectHandle(clientID,
                                                         'Pioneer_p3dx_rightMotor', vrep.simx_opmode_oneshot_wait)

sensor_handle_list = []
generated_x_map = []
generated_y_map = []

# Posicionamento angular dos sensores
sensor_angle = np.array([90, 50, 30, 10, -10, -30, -50, -90])
sensor_angle = sensor_angle * PI / 180.0

# Posicionamento do robo
returnCode, position = vrep.simxGetObjectPosition(
    clientID, robot_handle, -1, vrep.simx_opmode_streaming)

returnCode, eulerAngles = vrep.simxGetObjectOrientation(
    clientID, robot_handle, -1, vrep.simx_opmode_streaming)

# Primeira leitura dos handles
for x in range(0, 8):
    errorCode, sensor_handle = vrep.simxGetObjectHandle(clientID,
                                                        'Pioneer_p3dx_ultrasonicSensor' + str(x + 1), vrep.simx_opmode_oneshot_wait)
    sensor_handle_list.append(sensor_handle)
    errorCode, detectionState, detectedPoint, detectedObjectHandle, detectedSurfaceNormalVector = vrep.simxReadProximitySensor(
        clientID, sensor_handle, vrep.simx_opmode_streaming)
    returnCode, position = vrep.simxGetObjectPosition(
        clientID, sensor_handle, robot_handle, vrep.simx_opmode_streaming)


# Pega o posicionamento relativo de cada sonar ao robo
sensor_position = []
for x in range(0, 8):
    while (True):
        returnCode, position = vrep.simxGetObjectPosition(
            clientID, sensor_handle_list[x], robot_handle, vrep.simx_opmode_buffer)
        if (returnCode == 0):
            break
    sensor_position.append((position[0], position[1]))


def setVelocity(left, right):
    vrep.simxSetJointTargetVelocity(
        clientID, left_motor_handle, left, vrep.simx_opmode_streaming)
    vrep.simxSetJointTargetVelocity(
        clientID, right_motor_handle, right, vrep.simx_opmode_streaming)

setVelocity(2, 2)
t = time.time()
turning_left = False
turning_right = False

a_list = []
_thread.start_new_thread(input_thread, (a_list,))

while not a_list:
# while (time.time() - t) < 60:

    sensor_val = []

    # Faz leitrua dos sonares a cada iteração
    for x in range(1, 8 + 1):

        # Distancia detectada pelo sonar
        errorCode, detectionState, detectedPoint, detectedObjectHandle, detectedSurfaceNormalVector = vrep.simxReadProximitySensor(
            clientID, sensor_handle_list[x - 1], vrep.simx_opmode_buffer)
        distance = np.linalg.norm(detectedPoint)

        # Posicao global do robo
        returnCode, position = vrep.simxGetObjectPosition(
            clientID, robot_handle, -1, vrep.simx_opmode_buffer)
        returnCode, eulerAngles = vrep.simxGetObjectOrientation(
            clientID, robot_handle, -1, vrep.simx_opmode_buffer)

        if (detectionState == True):

            sensor_val.append(distance)

            # Coordadena do obstaculo é dado pela posicão global do
            # sonar (global robo + relativo sonar) + a projeção
            # da distancia detectada nos eixos
            x_obstacle = position[0] + (sensor_position[x-1][0] + distance) * (np.cos(eulerAngles[2]+sensor_angle[x-1]))
            y_obstacle = position[1] + (sensor_position[x-1][1] + distance) * (np.sin(eulerAngles[2]+sensor_angle[x-1]))
            # y_obstacle = detectedPoint[1] + position[1]
            # x_obstacle = detectedPoint[0] + position[0]

            generated_x_map.append(x_obstacle)
            generated_y_map.append(y_obstacle)

        else:
            sensor_val.append(np.inf)

    # Pega o menor valor dos sensores
    # min_ind = np.where(np.array(sensor_val) == np.min(sensor_val))
    # min_ind = min_ind[0][0]

    # se o menor valor é dos sensores frontais
    # if (sensor_val[min_ind] < 0.5) and (0 < min_ind < 7):
    #     if (0 < min_ind <= 3) and not turning_right:
    #         setVelocity(1, -1)
    #         turning_left = True
    #     elif (4 <= min_ind < 7) and not turning_left:
    #         setVelocity(-1, 1)
    #         turning_right = True
    # else:
    #     setVelocity(2, 2)
    #     turning_left = False
    #     turning_right = False

    act_dist = 0.25

    cont_left = 0
    cont_right = 0

    for i in range(1, 7):
        if i < 4:
            if sensor_val[i] < act_dist:
                cont_left += 1
        else:
            if sensor_val[i] < act_dist:
                cont_right += 1

    # if sensor_val[3] < act_dist:
    #     setVelocity(1, -1)
    # elif sensor_val[4] < act_dist:
    #     setVelocity(-1, 1)
    # elif sensor_val[2] < act_dist:
    #     setVelocity(1, -1)
    # elif sensor_val[5] < act_dist:
    #     setVelocity(-1, 1)
    if sensor_val[2] > act_dist and sensor_val[3] > act_dist and sensor_val[4] > act_dist and sensor_val[5] > act_dist:
        setVelocity(3, 3)
    elif cont_left > cont_right:
        setVelocity(2, -2) 
    else:
        setVelocity(-2, 2)

    # plt.scatter(generated_x_map, generated_y_map, s=2)
    # plt.show()
    # Considere execucao do loop a cada 0.2 segundos (= 5 Hz)
    time.sleep(0.2)

plt.scatter(generated_x_map, generated_y_map, s=2)
plt.show()

setVelocity(0, 0)
