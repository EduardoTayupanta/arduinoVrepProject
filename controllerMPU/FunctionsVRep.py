import cv2
import vrep
import time
import numpy as np


class FunctionsVRep:
    def __init__(self, clientID):
        self.clientID = clientID

    def start_simultion(self):
        _ = vrep.simxStartSimulation(self.clientID, vrep.simx_opmode_oneshot)

    def stop_simulation(self):
        _ = vrep.simxStopSimulation(self.clientID, vrep.simx_opmode_oneshot)

    def reference_handle(self, objectName):
        return vrep.simxGetObjectHandle(self.clientID, objectName, vrep.simx_opmode_blocking)

    def opmode_first_call(self, objectHandle):
        _, orientation = vrep.simxGetObjectOrientation(self.clientID, objectHandle, -1, vrep.simx_opmode_streaming)
        time.sleep(1)
        return orientation

    def opmode_first_call_cam(self, objectName):
        _, resolution, image = vrep.simxGetVisionSensorImage(self.clientID, objectName, 0, vrep.simx_opmode_streaming)
        time.sleep(1)
        return resolution, image

    def get_orientation(self, objectHandle):
        _, orientation = vrep.simxGetObjectOrientation(self.clientID, objectHandle, -1, vrep.simx_opmode_buffer)
        return orientation

    def set_orientation(self, objectHandle, orientation):
        vrep.simxSetObjectOrientation(self.clientID, objectHandle, -1, orientation, vrep.simx_opmode_oneshot)

    def get_image(self, objectHandle):
        _, resolution, image = vrep.simxGetVisionSensorImage(self.clientID, objectHandle, 0, vrep.simx_opmode_buffer)
        img = np.array(image, dtype=np.uint8)
        img.resize([resolution[0], resolution[1], 3])
        img = np.rot90(img, 2)
        img = np.fliplr(img)
        img = cv2.cvtColor(img, cv2.COLOR_RGB2BGR)
        return img
