import vrep
from person import Person

print('Program started')
vrep.simxFinish(-1)  # just in case, close all opened connections
clientID = vrep.simxStart('127.0.0.1', 19997, True, True, 5000, 5)  # Connect to V-REP
if clientID != -1:
    print('Connected to remote API server')
else:
    print('Failed connecting to remote API server')

p1 = Person(clientID, 'COM3')
p1.start()

print('Program ended')
