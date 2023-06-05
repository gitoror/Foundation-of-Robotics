import vrep
import time
import matplotlib.pyplot as plt
import numpy as np
from math import  sqrt, sin, cos
from trajectory import *

# simulation config
ip = '127.0.0.1'
port = 19997
scene = './planar_two_link.ttt'  # charge modele robot
# ce modele ne gere pas les collisions entre les différents corps  du robot

print ('Program started')
vrep.simxFinish(-1) # just in case, close all opened connections
client_id=vrep.simxStart(ip,port,True,True,5000,5) # Connect to V-REP

def jacobienne(q, L_1=1, L_2=1):
    return np.array([[-L_1 * sin(q[0]) - L_2 * sin(q[0] + q[1]), -L_2 * sin(q[0] + q[1])],
                     [L_1 * cos(q[0]) + L_2 * cos(q[0] + q[1]), L_2 * cos(q[0] + q[1])]])

def MCI_controller(dX, q, L_1=1, L_2=1):
    return np.linalg.pinv(jacobienne(q, L_1, L_2)).dot(dX)

if client_id!=-1:
    print ('Connected to remote   API server on %s:%s  (Vrep)' % (ip, port))
    res = vrep.simxLoadScene(client_id, scene, 1, vrep.simx_opmode_oneshot_wait)
    err_codeR, gripper = vrep.simxGetObjectHandle(client_id, 'Endeffector', vrep.simx_opmode_oneshot_wait)
    err_codeR, cible = vrep.simxGetObjectHandle(client_id, 'Cible', vrep.simx_opmode_oneshot_wait)
    err_code0, q0 = vrep.simxGetObjectHandle(client_id, 'joint1', vrep.simx_opmode_oneshot_wait)
    err_code1, q1 = vrep.simxGetObjectHandle(client_id, 'joint2', vrep.simx_opmode_oneshot_wait)
    vrep.simxStartSimulation(client_id, vrep.simx_opmode_blocking)
    
    t = 0   #temps
    n_points = 100
    dt = 0.1 # pas de temps = pas de calcul = pas de simulation 
    running = True
    epsilon = 1e-2 * 5 # tolerance pour la convergence      
          
    q = [0.5, 0.5]  # angular position control vector
    

    X = [] # coordonnées cartésiennes gripper
    Y = [] 
    D = [] 
    T = []  # time
    Target = [] # coordonnées cartesiennes cible  
    
    # coordonnées de la cible
    Tx,Ty,Tz = 1, -1, 0      
    
    print('go to configuration initiale')
    vrep.simxSetJointPosition(client_id, q0, q[0], vrep.simx_opmode_oneshot_wait)
    vrep.simxSetJointPosition(client_id, q1, q[1], vrep.simx_opmode_oneshot_wait)
    vrep.simxSetObjectPosition(client_id, cible, -1, (Tx,Ty,Tz), vrep.simx_opmode_oneshot_wait)

    res, tmp = vrep.simxGetObjectPosition(client_id, gripper, -1, vrep.simx_opmode_oneshot_wait)
    start_position = (tmp[0], tmp[1])
    print("start position: ", start_position)
    end_position = (Tx, Ty)
    distance_euclidienne = sqrt((Tx - tmp[0]) ** 2 + (Ty - tmp[1]) ** 2)
    D.append(distance_euclidienne)
    trajectory = generate_polynomial_trajectory(start_position, end_position, n_points=n_points)
    trajectory_x = trajectory[:,0]
    trajectory_y = trajectory[:,1]

    fig, (plot_desired_trajectory, plot_trayectory) = plt.subplots(2, 1)
    fig.tight_layout(pad=2.5)

    print("start movement")    
    t1 = time.time()
    t2 = time.time()
    t = 0
    T.append(t)
    t_limit = 100

    for i, point in enumerate(trajectory):
        res, tmp = vrep.simxGetObjectPosition(client_id, gripper, -1, vrep.simx_opmode_oneshot_wait)
        delta_x = np.array([point[0] - tmp[0], point[1] - tmp[1]])
        distance = np.linalg.norm(delta_x)
        distance_euclidienne = sqrt((Tx - tmp[0]) ** 2 + (Ty - tmp[1]) ** 2)
        D.append(distance_euclidienne)
        t2 = time.time()
        t += t2 - t1
        t1 = t2
        T.append(t)

        while distance > epsilon:
            dq = MCI_controller(delta_x,q)
            q[0] = q[0] + dq[0]*dt   #application de la vitesse articulaire
            q[1] = q[1] + dq[1]*dt 

            vrep.simxSetJointPosition(client_id, q0, q[0], vrep.simx_opmode_oneshot_wait)
            vrep.simxSetJointPosition(client_id, q1, q[1], vrep.simx_opmode_oneshot_wait)   

            res, tmp = vrep.simxGetObjectPosition(client_id, gripper, -1, vrep.simx_opmode_oneshot_wait)
            delta_x = np.array([point[0] - tmp[0], point[1] - tmp[1]])
            distance = np.linalg.norm(delta_x)
            distance_euclidienne = sqrt((Tx - tmp[0]) ** 2 + (Ty - tmp[1]) ** 2)
            D.append(distance_euclidienne)
            t2 = time.time()
            t += t2 - t1
            t1 = t2
            T.append(t)
            X.append(tmp[0]) # x cartesian
            Y.append(tmp[1]) # y cartesian


    plot_desired_trajectory.plot(trajectory_x, trajectory_y)
    plot_desired_trajectory.set_title("desired trajectory")
    plot_desired_trajectory.set_xlabel("x")
    plot_desired_trajectory.set_ylabel("y")
    plot_desired_trajectory.invert_xaxis()
    plot_desired_trajectory.invert_yaxis()

    plot_trayectory.plot(X,Y)
    plot_trayectory.set_title("trajectory")
    plot_trayectory.set_xlabel("x")
    plot_trayectory.set_ylabel("y")
    plot_trayectory.invert_xaxis()
    plot_trayectory.invert_yaxis()

    plt.show()

    plt.clf()
    plt.plot(D,T)
    plt.title("Euclidean Distance vs time")
    plt.xlabel("t")
    plt.ylabel("distance")
    plt.show()
    