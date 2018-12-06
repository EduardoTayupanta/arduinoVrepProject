import cv2
import math
import numpy as np
import time
import vrep


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
        _, position = vrep.simxGetJointPosition(self.clientID, objectHandle, vrep.simx_opmode_streaming)
        time.sleep(1)
        return position

    def opmode_first_call_image(self, objectHandle):
        _, resolution, image = vrep.simxGetVisionSensorImage(self.clientID, objectHandle, 0, vrep.simx_opmode_streaming)
        time.sleep(1)
        return resolution, image

    def get_position(self, objectHandle):
        _, position = vrep.simxGetJointPosition(self.clientID, objectHandle, vrep.simx_opmode_buffer)
        return int((position + math.pi / 2) * 180 / math.pi)

    def get_image(self, objectHandle):
        _, resolution, image = vrep.simxGetVisionSensorImage(self.clientID, objectHandle, 0, vrep.simx_opmode_buffer)
        img = np.array(image, dtype=np.uint8)
        img.resize([resolution[0], resolution[1], 3])
        img = np.rot90(img, 2)
        img = np.fliplr(img)
        img = cv2.cvtColor(img, cv2.COLOR_RGB2BGR)
        return img

    def set_joint_target_position(self, objectHandle, position):
        targetPosition = (position - 90) * math.pi / 180
        _ = vrep.simxSetJointTargetPosition(self.clientID, objectHandle, targetPosition, vrep.simx_opmode_streaming)
