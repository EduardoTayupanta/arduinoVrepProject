import vrep
from robot import Robot

print('Program started')
vrep.simxFinish(-1)  # just in case, close all opened connections
clientID = vrep.simxStart('127.0.0.1', 19997, True, True, 5000, 5)  # Connect to V-REP
if clientID != -1:
    print('Connected to remote API server')
else:
    print('Failed connecting to remote API server')

# [RightArm, LeftArm, RightLeg, LeftLeg]
p1 = Robot(clientID, True, 'COM3', 9600, [1, 1, 0, 0])
p1.start()
