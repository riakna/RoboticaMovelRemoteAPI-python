
try:
    import vrep
except:
    print ('--------------------------------------------------------------')
    print ('"vrep.py" could not be imported. This means very probably that')
    print ('either "vrep.py" or the remoteApi library could not be found.')
    print ('Make sure both are in the same folder as this file,')
    print ('or appropriately adjust the file "vrep.py"')
    print ('--------------------------------------------------------------')
    print ('')

import time
import cv2
import numpy as np
import matplotlib.pyplot as plt
import math

PI = math.pi

print("Simulação iniciada!!")
vrep.simxFinish(-1) #fecha todas as conexões existentes

port = 25100

clientID = vrep.simxStart('127.0.0.1', port, True, True, 5000, 5)

if clientID != -1:
    print("Conectado ao servidor remote API na porta ", port)

    errCode, left_motor_handle = vrep.simxGetObjectHandle(clientID,'Pioneer_p3dx_leftMotor',vrep.simx_opmode_oneshot_wait)

    if errCode != vrep.simx_return_ok:
        print("Erro em obter o handle")
    
    errCode, right_motor_handle = vrep.simxGetObjectHandle(clientID, 'Pioneer_p3dx_rightMotor', vrep.simx_opmode_oneshot_wait)

    if errCode != vrep.simx_return_ok:
        print("Erro em obter o handle")

    errCode, robot_handle = vrep.simxGetObjectHandle(clientID, 'Pioneer_p3dx', vrep.simx_opmode_oneshot_wait)

    if errCode != vrep.simx_return_ok:
        print("Erro em obter o handle")

    errCode, cam = vrep.simxGetObjectHandle(clientID, 'camera', vrep.simx_opmode_oneshot_wait)
    if errCode != vrep.simx_return_ok:
        print("Erro em obter o handle")


    # Armazena o posicionamento angular dos sensores
    sensor_angle = np.array([PI/2, 50/180.0*PI, 30/180.0*PI, 10/180.0*PI, 
                       -10/180.0*PI, -30/180.0*PI, -50/180.0*PI, -PI/2, -PI/2, 
                       -130/180.0*PI, -150/180.0*PI, -170/180.0*PI, 170/180.0*PI,
                       150/180.0*PI, 130/180.0*PI, PI/2])

    # recupera a posição do robô
    returnCode, position = vrep.simxGetObjectPosition(
        clientID, robot_handle, -1, vrep.simx_opmode_streaming)

    returnCode, eulerAngles = vrep.simxGetObjectOrientation(
        clientID, robot_handle, -1, vrep.simx_opmode_streaming)

    startTime = time.time()

    sensor_handle_list = []

    for i in range(16):
        errCode, sensor_handle = vrep.simxGetObjectHandle(clientID,'Pioneer_p3dx_ultrasonicSensor'+str(i+1), vrep.simx_opmode_oneshot_wait)
        sensor_handle_list.append(sensor_handle)
        errCode, detectionState, detectedPoint, detectedObjectHandle, detectedSurfaceNormalVector = vrep.simxReadProximitySensor(clientID, sensor_handle, vrep.simx_opmode_streaming) 
        errCode, position = vrep.simxGetObjectPosition(clientID, sensor_handle, robot_handle, vrep.simx_opmode_streaming)


    errCode, signalValue = vrep.simxGetStringSignal(clientID, "measuredDataAtThisTime",vrep.simx_opmode_streaming)
    measuredData=vrep.simxUnpackFloats(signalValue)

    def setVelocity(v_left, v_right):
        vrep.simxSetJointTargetVelocity(clientID, left_motor_handle, v_left, vrep.simx_opmode_streaming)
        vrep.simxSetJointTargetVelocity(clientID, right_motor_handle, v_right, vrep.simx_opmode_streaming)

    sensor_position_to_robot = []
    # Recupera a posição relativa de cada sensor para a posição do robô
    for i in range(16):
        while True:
            errCode, position = vrep.simxGetObjectPosition(clientID, sensor_handle_list[i], robot_handle, vrep.simx_opmode_buffer)
            if errCode == 0:
                break
        sensor_position_to_robot.append((position[0], position[1]))

    map_x = []
    map_y = []

    gt_x = []
    gt_y = []

    odom_x = []
    odom_y = []

    x_laser = []
    y_laser = []



    #number_of_turns = 0
    while vrep.simxGetConnectionId(clientID)!=-1:
        sensor_dist_values = []
        sensor_points_to_robot = []

        errCode, signalValue = vrep.simxGetStringSignal(clientID, "measuredDataAtThisTime",vrep.simx_opmode_buffer)
        measuredData=vrep.simxUnpackFloats(signalValue)

        laser_array = np.reshape(measuredData, (int(len(measuredData)/3),3))

        errCode, position = vrep.simxGetObjectPosition(clientID, robot_handle, -1, vrep.simx_opmode_buffer)
        errCode, eulerAngles = vrep.simxGetObjectOrientation(clientID, robot_handle, -1, vrep.simx_opmode_buffer)

        robot_angle = math.radians(math.degrees(eulerAngles[2])%360.0)
        print(math.degrees(robot_angle))

        x_laser = np.append(x_laser, laser_array[:,0]*np.cos(robot_angle)-laser_array[:,1]*np.sin(robot_angle)+position[0])
        y_laser = np.append(y_laser, laser_array[:,0]*np.sin(robot_angle)+laser_array[:,1]*np.cos(robot_angle)+position[1])


        plt.scatter(x_laser, y_laser, s=2)
        plt.pause(0.05)

        for i in range(1,16+1):
            errCode, detectionState, detectedPoint, detectedObjectHandle, detectedSurfaceNormalVector = vrep.simxReadProximitySensor(clientID, sensor_handle_list[i-1], vrep.simx_opmode_buffer)

            gt_x.append(position[0])
            gt_y.append(position[1])

            if detectionState == True:
                distance = np.linalg.norm(detectedPoint)
                sensor_dist_values.append(distance)

                if distance < 0.5 and distance >= 0.2:
                    x_obs_robot = detectedPoint[2]*np.cos(sensor_angle[i-1])-detectedPoint[0]*np.sin(sensor_angle[i-1])+sensor_position_to_robot[i-1][0]
                    y_obs_robot = detectedPoint[2]*np.sin(sensor_angle[i-1])-detectedPoint[0]*np.cos(sensor_angle[i-1])+sensor_position_to_robot[i-1][1]
                    sensor_points_to_robot.append((x_obs_robot, y_obs_robot))

                    x_robot_world = x_obs_robot*np.cos(eulerAngles[2])-y_obs_robot*np.sin(eulerAngles[2])+position[0]
                    y_robot_world = x_obs_robot*np.sin(eulerAngles[2])-y_obs_robot*np.cos(eulerAngles[2])+position[1]
                    map_x.append(x_robot_world)
                    map_y.append(y_robot_world)
            else:
                sensor_dist_values.append(np.inf)
                sensor_points_to_robot.append((0,0))
        
        setVelocity(2, 2)

        if sensor_dist_values[4] < 0.4 or sensor_dist_values[5] < 0.4 or sensor_dist_values[3] < 0.4 or sensor_dist_values[2] < 0.4:
            setVelocity(-1, -2)#vira a esquerda
            time.sleep(2)

        '''
        if sensor_dist_values[7] < 0.3:
            setVelocity(-1, 1)#vira a esquerda
            print("muito perto",sensor_dist_values[7], sensor_dist_values[8])
            time.sleep(0.6)
            setVelocity(1, 1)
            time.sleep(0.8)

        
        elif sensor_dist_values[7] > 0.5:

            print("muito longe",sensor_dist_values[7], sensor_dist_values[8])
            setVelocity(1, -1)
            time.sleep(0.6)
            setVelocity(1, 1)
            time.sleep(0.8)

        elif sensor_dist_values[4] < 0.2 or sensor_dist_values[5] < 0.2 or sensor_dist_values[3] < 0.2 or sensor_dist_values[2] < 0.2:
            setVelocity(-1, -1)
            time.sleep(1)
            setVelocity(-1, 1)
            time.sleep(3)
            print("obstaculo a frente",sensor_dist_values[4], sensor_dist_values[5])
        else:
            setVelocity(2, 2)

        '''
    # Before closing the connection to V-REP, make sure that the last command sent out had time to arrive. You can guarantee this with (for example):
    '''
    fig = plt.figure()
    ax1 = fig.add_subplot(111)
    ax1.scatter(map_x, map_y, s=2)
    ax1.scatter(gt_x, gt_y, s=2, c='r')
    plt.show()
    '''
    plt.show()
    setVelocity(0, 0)
    vrep.simxGetPingTime(clientID)

    # Now close the connection to V-REP:
    vrep.simxFinish(clientID)

    cv2.destroyAllWindows()
else:
    print("Falha na conexão com o servidor remote API")