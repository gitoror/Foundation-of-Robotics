import vrep
import math
import numpy as np
from tqdm import tqdm
from sklearn.model_selection import train_test_split

# simulation config
ip = '127.0.0.1'
port = 19997
scene = './franka.ttt'  # charge modele robot
# ce modele ne gere pas les collisions entre les différents corps  du robot
N_JOINTS = 2
JOINTS = [1,3]
# N_JOINTS = 7

print ('Program started')
vrep.simxFinish(-1) # just in case, close all opened connections
client_id=vrep.simxStart(ip,port,True,True,5000,5) # Connect to V-REP


if client_id!=-1:
    print ('Connected to remote   API server on %s:%s  (Vrep)' % (ip, port))
    res = vrep.simxLoadScene(client_id, scene, 1, vrep.simx_opmode_oneshot_wait)
    err_codeR, connection_handler = vrep.simxGetObjectHandle(client_id, 'Franka_connection', vrep.simx_opmode_oneshot_wait)
    res, tmp = vrep.simxGetObjectPosition(client_id, connection_handler, -1, vrep.simx_opmode_oneshot_wait)

    joints_handlers = [ vrep.simxGetObjectHandle(client_id, 'Franka_joint'+str(i), vrep.simx_opmode_oneshot_wait)[1] for i in range(1,8) ]
    
    limits = np.array([[-165, 167], [-100, 55], [-165, 167], [-175, -3], [-165,167], [0, 216], [-165, 167]])

    
    N_SAMPLES = 10000

    X = np.zeros((N_SAMPLES, 6))
    Y = np.zeros((N_SAMPLES, N_JOINTS))


    for i in tqdm(range(N_SAMPLES),desc="Creating dataset..."):
        random_point = np.random.uniform(limits[:,0], limits[:,1], (1,7))
        for j in JOINTS:
            vrep.simxSetJointPosition(client_id, joints_handlers[j], math.pi * random_point[0][j] / 180, vrep.simx_opmode_oneshot_wait)
        
        
        final_position = vrep.simxGetObjectPosition(client_id, connection_handler, -1, vrep.simx_opmode_oneshot_wait)[1]
        final_orientation = vrep.simxGetObjectOrientation(client_id, connection_handler, -1, vrep.simx_opmode_oneshot_wait)[1]
        
        # print(final_orientation)
        # Check if the position is valid
        # while final_position[2] <= 0.07:
        #     random_point = np.random.uniform(limits[:,0], limits[:,1], (1,7))
        #     for j in range(N_JOINTS):
        #         vrep.simxSetJointPosition(client_id, joints_handlers[j], math.pi * random_point[0][j] / 180, vrep.simx_opmode_oneshot_wait)
        #         final_position = vrep.simxGetObjectPosition(client_id, connection_handler, -1, vrep.simx_opmode_oneshot_wait)[1]
        
        X[i] = final_position + final_orientation
        Y[i] = random_point[:,JOINTS]
    
# print('X: ', X)
# print('Y: ', Y)
    np.savetxt('data/X_TWO_JOINTS.csv', X, delimiter=',')
    np.savetxt('data/Y_TWO_JOINTS.csv', Y, delimiter=',')

    # X_train, X_test, Y_train, Y_test = train_test_split(X, Y, test_size=0.2, random_state=42)
    # np.savetxt('data/X_1_train.txt', X_train)
    # np.savetxt('data/Y_1_train.txt', Y_train)
    # np.savetxt('data/X_1_test.txt', X_test)
    # np.savetxt('data/Y_1_test.txt', Y_test)