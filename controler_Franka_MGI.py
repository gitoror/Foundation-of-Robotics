import vrep
import math
from tqdm import tqdm
import time
import matplotlib.pyplot as plt
import numpy as np
from math import atan2, acos, sqrt, pi
from tensorflow import keras
from trajectory import generate_polynomial_trajectory, plot_trajectory, generate_linear_trajectory, generate_radical_trajectory

# simulation config
ip = '127.0.0.1'
port = 19997
scene = './franka.ttt'  # charge modele robot
# ce modele ne gere pas les collisions entre les différents corps  du robot
N_JOINTS = 7

print ('Program started')
vrep.simxFinish(-1) # just in case, close all opened connections
client_id=vrep.simxStart(ip,port,True,True,5000,5) # Connect to V-REP

# Load Models:

model_inverse_1 = keras.models.load_model("models/model_inverse_1.h5")
model_inverse_2 = keras.models.load_model("models/model_inverse_2.h5")
model_inverse_3 = keras.models.load_model("models/model_inverse_3.h5")

# models = [model_inverse_1, model_inverse_2, model_inverse_3]
models = [model_inverse_1]

# Trajectories:

trajectory_1 = generate_linear_trajectory([0.0,1.0] , [0.6, 0.2] , 300)
trajectory_2 = generate_polynomial_trajectory([-0.3,1.05] , [0.7, 0.25], 8,300)
trajectory_3 = generate_polynomial_trajectory([-0.5,0.9], [0.2, 0.2], 8, 300)

# trajectories = [trajectory_1, trajectory_2, trajectory_3]
trajectories = [trajectory_1]

if client_id!=-1:
    print ('Connected to remote   API server on %s:%s  (Vrep)' % (ip, port))
    res = vrep.simxLoadScene(client_id, scene, 1, vrep.simx_opmode_oneshot_wait)
    err_codeR, connection_handler = vrep.simxGetObjectHandle(client_id, 'Franka_connection', vrep.simx_opmode_oneshot_wait)
    res, tmp = vrep.simxGetObjectPosition(client_id, connection_handler, -1, vrep.simx_opmode_oneshot_wait)

    joints_handlers = [ vrep.simxGetObjectHandle(client_id, 'Franka_joint'+str(i), vrep.simx_opmode_oneshot_wait)[1] for i in range(1,N_JOINTS + 1) ]
    
    limits = np.array([[-165, 167], [-100, 55], [-165, 167], [-175, -3], [-165,167], [0, 216], [-165, 167]])
    
    # Joints:
    j = 1 
    i = 3
    
    for m, model_inverse in enumerate(models):
        for t, trajectory in enumerate(trajectories):
            
            # Commands to follow the trajectories:
            trajectory_commands = model_inverse.predict(np.matrix(trajectory))

            followed_trajectory = []

            for command in tqdm(trajectory_commands, desc=f"Following Trajectory {t+1} with model {m+1}..."):
                vrep.simxSetJointPosition(client_id, joints_handlers[j], math.pi * command[0]  / 180, vrep.simx_opmode_oneshot_wait)
                vrep.simxSetJointPosition(client_id, joints_handlers[i], math.pi * command[1]  / 180, vrep.simx_opmode_oneshot_wait)
                # print("position: ", vrep.simxGetObjectPosition(client_id, connection_handler, -1, vrep.simx_opmode_oneshot_wait)[1])
                pos = vrep.simxGetObjectPosition(client_id, connection_handler, -1, vrep.simx_opmode_oneshot_wait)[1]
                followed_trajectory.append([pos[0],pos[2]])
            
            # Plotting:
            plt.title("Trajectory " + str(t+1) + " with model " + str(m+1))
            plt.xlabel("X")
            plt.ylabel("Z")
            plot_trajectory(trajectory, "Real Trajectory")
            plot_trajectory(np.array(followed_trajectory), "Followed Trajectory")
            plt.legend()
            plt.savefig("plots/trajectory_" + str(t+1) + "_model_" + str(m+1) + ".png")
            plt.clf()
            # plt.show()
