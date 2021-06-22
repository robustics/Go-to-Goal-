# Import the necessary the pacakges and test if sim located in the same directory
import numpy as np
import math
import time
try:
    import sim
except:
    print ('--------------------------------------------------------------')
    print ('"vrep.py" could not be imported. This means very probably that')
    print ('either "vrep.py" or the remoteApi library could not be found.')
    print ('Make sure both are in the same folder as this file,')
    print ('or appropriately adjust the file "vrep.py"')
    print ('--------------------------------------------------------------')
    print ('')


def r_pose():
    position = sim.simx_getObjectPosition(clientID,robot_pose,-1,)


def goToGoal(clientID):


if __name__ == "__main__":
    print('Prgram Started')
    sim.simxFinish(-1)
    clientID=sim.simxStart('127.0.0.1', 19997,True,True,5000,5)
    if clientID != -1:
        print('Connected to remote API server')
        ret = sim.simxStartSimulation(clientID, sim.simx_opmode_oneshot)
        ret,motorLeft = sim.simxGetObjectHandle(clientID,'Pioneer_p3dx_leftMotor', sim.simx_opmode_blocking)
        ret,motorRight = sim.simxGetObjectHandle(clientID,'Pioneer_p3dx_rightMotor', sim.simx_opmode_blocking)
        ret,target = sim.simxGetObjectHandle(clientID, 'Goal', sim.simx_opmode_blocking)
        ret,robot_pose = sim.simxGetObjectHandle(clientID,'Robot', sim.simx_opmode_blocking)
        goToGoal(clientID)

    else:
        print('Failed to connect to remote API server')
        exit(0)
