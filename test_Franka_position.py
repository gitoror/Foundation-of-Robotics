import vrep
import math
import numpy as np
from tqdm import tqdm
from sklearn.model_selection import train_test_split
from trajectory import generate_polynomial_trajectory, plot_trajectory
import matplotlib.pyplot as plt

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

    random_point = np.random.uniform(limits[:,0], limits[:,1], (1,7))
    # for j in range(N_JOINTS):
    j = 1
    i = 3
    # vrep.simxSetJointPosition(client_id, joints_handlers[0], math.pi * limits[0][0] / 180, vrep.simx_opmode_oneshot_wait)
    # vrep.simxSetJointPosition(client_id, joints_handlers[1], math.pi * -100 / 180, vrep.simx_opmode_oneshot_wait)
    # vrep.simxSetJointPosition(client_id, joints_handlers[2], math.pi * -165 / 180, vrep.simx_opmode_oneshot_wait)
    start_position = vrep.simxGetObjectPosition(client_id, connection_handler, -1, vrep.simx_opmode_oneshot_wait)[1]
    
    vrep.simxSetJointPosition(client_id, joints_handlers[j], math.pi * limits[j][0]  / 180, vrep.simx_opmode_oneshot_wait)
    vrep.simxSetJointPosition(client_id, joints_handlers[i], math.pi * limits[i][1]  / 180, vrep.simx_opmode_oneshot_wait)

    
    
    
    final_position = vrep.simxGetObjectPosition(client_id, connection_handler, -1, vrep.simx_opmode_oneshot_wait)[1]
    final_orientation = vrep.simxGetObjectOrientation(client_id, connection_handler, -1, vrep.simx_opmode_oneshot_wait)[1]
    # print("p1: ", final_position)
    # print("p2:", start_position)

    trajectory = generate_polynomial_trajectory(start_position,final_position,grade=2,n_points=1000)
    # plot_trajectory(trajectory)
    # plt.show()

    # print(final_orientation)