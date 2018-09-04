
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

print("Simulação iniciada!!")
vrep.simxFinish(-1) #fecha todas as conexões existentes

port = 25100

clientID = vrep.simxStart('127.0.0.1', port, True, True, 5000, 5)

if clientID != -1:
    print("Conectado ao servidor remote API na porta ", port)

    errCode, leftMotor = vrep.simxGetObjectHandle(clientID,'Pioneer_p3dx_leftMotor',vrep.simx_opmode_oneshot_wait)
    errCode, rightMotor = vrep.simxGetObjectHandle(clientID, 'Pioneer_p3dx_rightMotor', vrep.simx_opmode_oneshot_wait)

    #braitenbergL = [-0.2, -0.4, -0.6, -0.8, -1, -1.2, -1.4, -1.6, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
    #braitenbergR = [-1.6, -1.4, -1.2, -1, -0.8, -0.6, -0.4, -0.2, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
    v0 = 2

    res1 = vrep.simxSetJointTargetVelocity(clientID, leftMotor, 0.2, vrep.simx_opmode_streaming)
    res2 = vrep.simxSetJointTargetVelocity(clientID, rightMotor, 0.2, vrep.simx_opmode_streaming)



    # Before closing the connection to V-REP, make sure that the last command sent out had time to arrive. You can guarantee this with (for example):
    vrep.simxGetPingTime(clientID)

    # Now close the connection to V-REP:
    vrep.simxFinish(clientID)
else:
    print("Falha na conexão com o servidor remote API")