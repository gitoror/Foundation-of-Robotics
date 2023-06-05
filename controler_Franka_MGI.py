import vrep
import math
import time
import matplotlib.pyplot as plt
import numpy as np
from math import atan2, acos, sqrt, pi

# simulation config
ip = '127.0.0.1'
port = 19997
scene = './franka.ttt'  # charge modele robot
# ce modele ne gere pas les collisions entre les différents corps  du robot
N_JOINTS = 7

print ('Program started')
vrep.simxFinish(-1) # just in case, close all opened connections
client_id=vrep.simxStart(ip,port,True,True,5000,5) # Connect to V-REP


if client_id!=-1:
    print ('Connected to remote   API server on %s:%s  (Vrep)' % (ip, port))
    res = vrep.simxLoadScene(client_id, scene, 1, vrep.simx_opmode_oneshot_wait)
    err_codeR, connection_handler = vrep.simxGetObjectHandle(client_id, 'Franka_connection', vrep.simx_opmode_oneshot_wait)
    res, tmp = vrep.simxGetObjectPosition(client_id, connection_handler, -1, vrep.simx_opmode_oneshot_wait)

    joints_handlers = [ vrep.simxGetObjectHandle(client_id, 'Franka_joint'+str(i), vrep.simx_opmode_oneshot_wait)[1] for i in range(1,N_JOINTS + 1) ]
    
    limits = np.array([[-165, 167], [-100, 55], [-165, 167], [-175, -3], [-165,167], [0, 216], [-165, 167]])

    
    N_SAMPLES = 1000

    X = np.zeros((N_SAMPLES, N_JOINTS))

    random_point = np.random.uniform(limits[:,0], limits[:,1], (1,7))
    print("random_point: ", random_point)
    for i in range(7):
        vrep.simxSetJointPosition(client_id, joints_handlers[i], math.pi * random_point[0][i] / 180, vrep.simx_opmode_oneshot_wait)
    
    final_position = vrep.simxGetObjectPosition(client_id, connection_handler, -1, vrep.simx_opmode_oneshot_wait)[1]
    print("final position", final_position)
    # print('Initial cartesian position: ', tmp)
    
    # vrep.simxSetJointPosition(client_id, joints_handlers[0], math.pi/2, vrep.simx_opmode_oneshot_wait)
    # vrep.simxSetJointPosition(client_id, joints_handlers[1], -math.pi*0.47, vrep.simx_opmode_oneshot_wait) # math.pi/3.1 suelo hacia adelante, math.pi/2 suelo hacia atras
    # res, tmp = vrep.simxGetObjectPosition(client_id, connection_handler, -1, vrep.simx_opmode_oneshot_wait)
    # print('Final cartesian position: ', tmp)
    # for i in range(7):
    # lim_inf = -180
    # vrep.simxSetJointPosition(client_id, joints_handlers[i], math.pi * lim_inf / 180, vrep.simx_opmode_oneshot_wait) # math.pi/3.1 suelo hacia adelante, math.pi/2 suelo hacia atras
    # res, tmp = vrep.simxGetObjectPosition(client_id, connection_handler, -1, vrep.simx_opmode_oneshot_wait)
    # tmp = np.array(tmp)
    # anterior = tmp
    # epsilon = 1e-6
    # delta = 1e-1/2
    # while np.linalg.norm(tmp - anterior) <= epsilon:
    #     # print("lim_inf: ",lim_inf,np.linalg.norm(tmp - anterior) <= 1e-6)
    #     lim_inf += 1
    #     anterior = tmp
    #     vrep.simxSetJointPosition(client_id, joints_handlers[i], math.pi * lim_inf / 180, vrep.simx_opmode_oneshot_wait) # math.pi/3.1 suelo hacia adelante, math.pi/2 suelo hacia atras
    #     tmp = np.array(vrep.simxGetObjectPosition(client_id, connection_handler, -1, vrep.simx_opmode_oneshot_wait)[1])
    # lim_sup = lim_inf
    # # anterior = tmp
    # while np.linalg.norm(tmp - anterior) > epsilon and tmp[2] > delta:
    #     lim_sup += 1
    #     anterior = tmp
    #     vrep.simxSetJointPosition(client_id, joints_handlers[i], math.pi * lim_sup / 180, vrep.simx_opmode_oneshot_wait) # math.pi/3.1 suelo hacia adelante, math.pi/2 suelo hacia atras
    #     tmp = np.array(vrep.simxGetObjectPosition(client_id, connection_handler, -1, vrep.simx_opmode_oneshot_wait)[1])
    # print(f"join {i}: ",lim_inf,lim_sup)
    # X = [] # coordonnées cartésiennes gripper
    # Y = [] 
    # D = [] # distance gripper - cible 
    # T = []  # time

    # Target = [] # coordonnées cartesiennes cible  
    
    # # coordonnées de la cible
    # Tx,Ty,Tz = 1, -1, 0      
    
    # print('go to configuration initiale')
    # vrep.simxSetJointPosition(client_id, q0, q[0], vrep.simx_opmode_oneshot_wait)
    # vrep.simxSetJointPosition(client_id, q1, q[1], vrep.simx_opmode_oneshot_wait)
    # vrep.simxSetObjectPosition(client_id, cible, -1, (Tx,Ty,Tz), vrep.simx_opmode_oneshot_wait)

    # print("start movement")    
    
    # t1 = time.time()
    # t2 = time.time()
    
    # t = 0
    # t_limit = 80
    # dq = [0.5, 0.5] # angular velocity control vector
    # q_desired = MGI_controller(Tx,Ty)
    # delta_q = [q_desired[0] - q[0], q_desired[1] - q[1]]
    
    # res, tmp = vrep.simxGetObjectPosition(client_id, gripper, -1, vrep.simx_opmode_oneshot_wait)
    
    # distance = sqrt((Tx - tmp[0]) ** 2 + (Ty - tmp[1]) ** 2)
    
    # X.append(tmp[0]) # x cartesian
    # Y.append(tmp[1]) # y cartesian
    # D.append(distance) # distance gripper - cible
    # T.append(t) # time
    
    # while(distance > epsilon and t < t_limit): 
    #     t2 = time.time()
    #     t += t2 - t1
    #     t1 = t2
    #     # coordonnées de la cible   
    #     #  Commande espace articulaire
    #     q[0] = q[0] + delta_q[0] * dt   #application de la vitesse articulaire
    #     q[1] = q[1] + delta_q[1] * dt
              
    #     vrep.simxSetJointPosition(client_id, q0, q[0], vrep.simx_opmode_oneshot_wait)
    #     vrep.simxSetJointPosition(client_id, q1, q[1], vrep.simx_opmode_oneshot_wait)   
        
    #     res, tmp = vrep.simxGetObjectPosition(client_id, gripper, -1, vrep.simx_opmode_oneshot_wait)
    #     res, tmp1 = vrep.simxGetObjectPosition(client_id, cible, -1, vrep.simx_opmode_oneshot_wait)
        

    #     # Update delta and distance
    #     delta_q = [q_desired[0] - q[0], q_desired[1] - q[1]]
    #     distance = sqrt((Tx - tmp[0]) ** 2 + (Ty - tmp[1]) ** 2)

    #     X.append(tmp[0]) # x cartesian
    #     Y.append(tmp[1]) # y cartesian
    #     D.append(distance) # z cartesian
    #     T.append(t) # time

    #     Target.append(tmp1[0]);Target.append(tmp1[1]);Target.append(tmp1[2])
    
    # plt.plot(D,T)
    # plt.title("Distance vs temps")
    # plt.xlabel("t")
    # plt.ylabel("distance")
    # plt.show()
    # plt.clf()

    # pad = 0.1
    # plt.plot(X,Y)
    # plt.xlim(max(X)+pad, min(X)-pad)
    # plt.ylim(max(Y)+pad, min(Y)-pad)
    # plt.title("Followed cartesian trajectory")
    # plt.xlabel("x")
    # plt.ylabel("y")
    # plt.show()
    

