import vrep
import math
from tqdm import tqdm
import time
import matplotlib.pyplot as plt
import numpy as np
from math import atan2, acos, sqrt, pi
from tensorflow import keras
from trajectory import generate_polynomial_trajectory, plot_trajectory

# simulation config
ip = '127.0.0.1'
port = 19997
scene = './franka.ttt'  # charge modele robot
# ce modele ne gere pas les collisions entre les différents corps  du robot
N_JOINTS = 7

print ('Program started')
vrep.simxFinish(-1) # just in case, close all opened connections
client_id=vrep.simxStart(ip,port,True,True,5000,5) # Connect to V-REP

# Load Model:
model_inverse = keras.models.load_model("models/model_inverse_2joints_64hl.h5")

if client_id!=-1:
    print ('Connected to remote   API server on %s:%s  (Vrep)' % (ip, port))
    res = vrep.simxLoadScene(client_id, scene, 1, vrep.simx_opmode_oneshot_wait)
    err_codeR, connection_handler = vrep.simxGetObjectHandle(client_id, 'Franka_connection', vrep.simx_opmode_oneshot_wait)
    res, tmp = vrep.simxGetObjectPosition(client_id, connection_handler, -1, vrep.simx_opmode_oneshot_wait)

    joints_handlers = [ vrep.simxGetObjectHandle(client_id, 'Franka_joint'+str(i), vrep.simx_opmode_oneshot_wait)[1] for i in range(1,N_JOINTS + 1) ]
    
    limits = np.array([[-165, 167], [-100, 55], [-165, 167], [-175, -3], [-165,167], [0, 216], [-165, 167]])
    start_position = vrep.simxGetObjectPosition(client_id, connection_handler, -1, vrep.simx_opmode_oneshot_wait)[1]
    
    # Joints:
    j = 1 
    i = 3
    
    vrep.simxSetJointPosition(client_id, joints_handlers[j], math.pi * limits[j][0]  / 180, vrep.simx_opmode_oneshot_wait)
    vrep.simxSetJointPosition(client_id, joints_handlers[i], math.pi * limits[i][1]  / 180, vrep.simx_opmode_oneshot_wait)
    
    final_position = vrep.simxGetObjectPosition(client_id, connection_handler, -1, vrep.simx_opmode_oneshot_wait)[1]
    
    real_trajectory = generate_polynomial_trajectory(start_position,final_position,grade=2,n_points=100)

    # print(model_inverse.predict(np.matrix(real_trajectory)).shape)
    trajectory_commands = model_inverse.predict(np.matrix(real_trajectory))

    followed_trajectory = []

    for command in tqdm(trajectory_commands, desc="Following commands..."):
        vrep.simxSetJointPosition(client_id, joints_handlers[j], math.pi * command[0]  / 180, vrep.simx_opmode_oneshot_wait)
        vrep.simxSetJointPosition(client_id, joints_handlers[i], math.pi * command[1]  / 180, vrep.simx_opmode_oneshot_wait)
        followed_trajectory.append(vrep.simxGetObjectPosition(client_id, connection_handler, -1, vrep.simx_opmode_oneshot_wait)[1])
    
    plot_trajectory(real_trajectory)
    plot_trajectory(np.array(followed_trajectory))
    plt.show()
