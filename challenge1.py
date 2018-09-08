
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

print("Simulação iniciada!!")
vrep.simxFinish(-1) #fecha todas as conexões existentes

port = 25100

clientID = vrep.simxStart('127.0.0.1', port, True, True, 5000, 5)

if clientID != -1:
    print("Conectado ao servidor remote API na porta ", port)

    errCode, leftMotor = vrep.simxGetObjectHandle(clientID,'Pioneer_p3dx_leftMotor',vrep.simx_opmode_oneshot_wait)
    errCode, rightMotor = vrep.simxGetObjectHandle(clientID, 'Pioneer_p3dx_rightMotor', vrep.simx_opmode_oneshot_wait)
    errCode, cam = vrep.simxGetObjectHandle(clientID, 'camera', vrep.simx_opmode_oneshot_wait)

    startTime = time.time()

    res1 = vrep.simxSetJointTargetVelocity(clientID, leftMotor, 0, vrep.simx_opmode_streaming)
    res2 = vrep.simxSetJointTargetVelocity(clientID, rightMotor, 0.5, vrep.simx_opmode_streaming)

    errCode, resolution, image = vrep.simxGetVisionSensorImage(clientID, cam, 0, vrep.simx_opmode_streaming)

    #while time.time()-startTime < 5:
    while vrep.simxGetConnectionId(clientID)!=-1:
        res, resolution, image = vrep.simxGetVisionSensorImage(clientID, cam, 0, vrep.simx_opmode_buffer)
        if res == vrep.simx_return_ok:
            plt.figure()
            im = np.array(image, dtype=np.uint8)#Create a numpy array with uint8 type
            im.resize([resolution[1], resolution[0],3])#resize the array to the resolution

            #blur = cv2.blur(im, (5,5))
            #plt.imshow(blur, origin='lower', cmap='gray')
            #plt.show()
            edges = cv2.Canny(im,130, 300)

            plt.imshow(edges, origin='lower',  cmap='gray')
            plt.show()

            lines = cv2.HoughLinesP(edges, 1, np.pi / 180, 10, 100, 10)

            print(lines)
            if not (lines is None):
                for i in range(len(lines)):
                    #for rho, theta in lines[i]:

                       # a = np.cos(theta)
                       # b = np.sin(theta)
                        #x0 = a * rho
                        #y0 = b * rho
                       # x1 = int(x0 + 1000 * (-b))
                        #y1 = int(y0 + 1000 * (a))
                       # x2 = int(x0 - 1000 * (-b))
                        #y2 = int(y0 - 1000 * (a))

                        #cv2.line(im, (x1, y1), (x2, y2), (0, 0, 255), 2)
                        

                    for x1, y1, x2, y2 in lines[i]:
                        cv2.line(im, (x1, y1), (x2, y2), (0, 255, 0), 2)

                plt.imshow(im, origin='lower')
                plt.show()


    # Before closing the connection to V-REP, make sure that the last command sent out had time to arrive. You can guarantee this with (for example):
    vrep.simxGetPingTime(clientID)

    # Now close the connection to V-REP:
    vrep.simxFinish(clientID)
else:
    print("Falha na conexão com o servidor remote API")