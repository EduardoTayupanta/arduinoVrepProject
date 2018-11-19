from threading import Thread
import cv2
import vrep
import time
import math
import numpy as np
import serial


class Person(Thread):
    def __init__(self, clientID, COM):
        ''' Constructor. '''
        Thread.__init__(self)
        self.clientID = clientID

        # Init Simulation
        self.start_simultion()

        # Reference of the elements
        _, self.cam_handle = self.reference_handle('Vision_sensor')

        _, self.right_arm_handle = self.reference_handle('Joint10')
        _, self.right_forearm_handle = self.reference_handle('Joint0')

        _, self.left_arm_handle = self.reference_handle('Joint9')
        _, self.left_forearm_handle = self.reference_handle('Joint')

        _, self.right_thigh_handle = self.reference_handle('Joint12')
        _, self.right_leg_handle = self.reference_handle('Joint6')

        _, self.left_thigh_handle = self.reference_handle('Joint11')
        _, self.left_leg_handle = self.reference_handle('Joint5')

        # Start the elements and wait a second to fill the buffer
        # position and orientation of the right arm
        self.Rt_right_arm = self.opmode_first_call(self.right_arm_handle)
        # position and orientation of the right forearm
        self.Rt_right_forearm = self.opmode_first_call(self.right_forearm_handle)

        # position and orientation of the left arm
        self.Rt_left_arm = self.opmode_first_call(self.left_arm_handle)
        # position and orientation of the left forearm
        self.Rt_left_forearm = self.opmode_first_call(self.left_forearm_handle)

        # position and orientation of the right thigh
        self.Rt_right_thigh = self.opmode_first_call(self.right_thigh_handle)
        # position and orientation of the right leg
        self.Rt_right_leg = self.opmode_first_call(self.right_leg_handle)

        # position and orientation of the left thigh
        self.Rt_left_thigh = self.opmode_first_call(self.left_thigh_handle)
        # position and orientation of the left leg
        self.Rt_left_leg = self.opmode_first_call(self.left_leg_handle)

        # resolution and image of the cam
        _, _ = self.opmode_first_call_cam(self.cam_handle)

        # Init communication arduino
        self.arduino = self.start_communication_arduino(COM)

    def nothing(self, *arg):
        pass

    def start_simultion(self):
        _ = vrep.simxStartSimulation(self.clientID, vrep.simx_opmode_oneshot)

    def stop_simulation(self):
        _ = vrep.simxStopSimulation(self.clientID, vrep.simx_opmode_oneshot)

    def reference_handle(self, objectName):
        return vrep.simxGetObjectHandle(self.clientID, objectName, vrep.simx_opmode_blocking)

    def opmode_first_call(self, objectHandle):
        _, position = vrep.simxGetObjectPosition(self.clientID, objectHandle, -1, vrep.simx_opmode_streaming)
        time.sleep(1)
        _, orientation = vrep.simxGetObjectOrientation(self.clientID, objectHandle, -1, vrep.simx_opmode_streaming)
        time.sleep(1)
        return self.eulerAnglesToRotationMatrix(position, orientation)

    def opmode_first_call_cam(self, objectName):
        _, resolution, image = vrep.simxGetVisionSensorImage(self.clientID, objectName, 0, vrep.simx_opmode_streaming)
        time.sleep(1)
        return resolution, image

    def get_position_orientation(self, objectHandle):
        _, position = vrep.simxGetObjectPosition(self.clientID, objectHandle, -1, vrep.simx_opmode_buffer)
        _, orientation = vrep.simxGetObjectOrientation(self.clientID, objectHandle, -1, vrep.simx_opmode_buffer)
        return self.eulerAnglesToRotationMatrix(position, orientation)

    def set_position_orientation(self, objectHandle, Rt_simlulation, Rt_arduino):
        vector_aux = np.array([[0, 0, 0, 1]])
        rt_s = np.vstack((Rt_simlulation, vector_aux))
        rt_a = np.vstack((Rt_arduino, vector_aux))

        R = np.dot(rt_a, rt_s)

        position = R[:3, 3].T
        orientation = self.rotationMatrixToEulerAngles(R[:3, :3])

        # vrep.simxSetObjectPosition(self.clientID, objectHandle, -1, position, vrep.simx_opmode_oneshot)
        vrep.simxSetObjectOrientation(self.clientID, objectHandle, -1, orientation, vrep.simx_opmode_oneshot)

    def get_image(self, objectHandle):
        _, resolution, image = vrep.simxGetVisionSensorImage(self.clientID, objectHandle, 0, vrep.simx_opmode_buffer)
        img = np.array(image, dtype=np.uint8)
        img.resize([resolution[0], resolution[1], 3])
        img = np.rot90(img, 2)
        img = np.fliplr(img)
        img = cv2.cvtColor(img, cv2.COLOR_RGB2BGR)
        return img

    # Calculates Rotation Matrix given euler angles.
    def eulerAnglesToRotationMatrix(self, position, orientation):
        R_x = np.array([[1, 0, 0],
                        [0, math.cos(orientation[0]), -math.sin(orientation[0])],
                        [0, math.sin(orientation[0]), math.cos(orientation[0])]])

        R_y = np.array([[math.cos(orientation[1]), 0, math.sin(orientation[1])],
                        [0, 1, 0],
                        [-math.sin(orientation[1]), 0, math.cos(orientation[1])]])

        R_z = np.array([[math.cos(orientation[2]), -math.sin(orientation[2]), 0],
                        [math.sin(orientation[2]), math.cos(orientation[2]), 0],
                        [0, 0, 1]])

        R = np.dot(R_z, np.dot(R_y, R_x))
        T = np.array([[position[0], position[1], position[2]]]).reshape(3, 1)
        Rt = np.hstack((R, T))

        return Rt

    # Checks if a matrix is a valid rotation matrix.
    def isRotationMatrix(self, R):
        Rt = np.transpose(R)
        shouldBeIdentity = np.dot(Rt, R)
        I = np.identity(3, dtype=R.dtype)
        n = np.linalg.norm(I - shouldBeIdentity)
        return n < 1e-6

    # Calculates rotation matrix to euler angles
    # The result is the same as MATLAB except the order
    # of the euler angles ( x and z are swapped ).
    def rotationMatrixToEulerAngles(self, R):

        assert (self.isRotationMatrix(R))

        sy = math.sqrt(R[0, 0] * R[0, 0] + R[1, 0] * R[1, 0])

        singular = sy < 1e-6

        if not singular:
            x = math.atan2(R[2, 1], R[2, 2])
            y = math.atan2(-R[2, 0], sy)
            z = math.atan2(R[1, 0], R[0, 0])
        else:
            x = math.atan2(-R[1, 2], R[1, 1])
            y = math.atan2(-R[2, 0], sy)
            z = 0

        return np.array([x, y, z])

    def start_communication_arduino(self, COM):
        return serial.Serial(COM, 9600)

    def run(self):
        aux = 0

        # help graph
        icol = (0, 0, 0)
        cv2.namedWindow('Angles')
        cv2.createTrackbar('alpha', 'Angles', icol[0], 360, self.nothing)
        cv2.createTrackbar('beta', 'Angles', icol[1], 360, self.nothing)
        cv2.createTrackbar('gamma', 'Angles', icol[2], 360, self.nothing)

        while 1:
            # Save frame of the camera, rotate it and convert it to BGR
            img = self.get_image(self.cam_handle)

            # Real-time position
            self.Rt_right_arm = self.get_position_orientation(self.right_arm_handle)
            self.Rt_right_forearm = self.get_position_orientation(self.right_forearm_handle)

            self.Rt_left_arm = self.get_position_orientation(self.left_arm_handle)
            self.Rt_left_forearm = self.get_position_orientation(self.left_forearm_handle)

            self.Rt_right_thigh = self.get_position_orientation(self.right_thigh_handle)
            self.Rt_right_leg = self.get_position_orientation(self.right_leg_handle)

            self.Rt_left_thigh = self.get_position_orientation(self.left_thigh_handle)
            self.Rt_left_leg = self.get_position_orientation(self.left_leg_handle)

            # alpha = cv2.getTrackbarPos('alpha', 'Angles')
            # beta = cv2.getTrackbarPos('beta', 'Angles')
            # gamma = cv2.getTrackbarPos('gamma', 'Angles')

            # cv2.setTrackbarPos('alpha', 'Angles', int(orientation[0] * 180 / math.pi))
            # cv2.setTrackbarPos('beta', 'Angles', int(orientation[1] * 180 / math.pi))
            # cv2.setTrackbarPos('gamma', 'Angles', int(orientation[2] * 180 / math.pi))
            #
            rawString = self.arduino.readline()
            print(rawString)
            # for item in rawString:
            #     print(item)

            # if aux == 0:
            #     R_a = np.array([[math.cos(-math.pi/9), 0, math.sin(-math.pi/9), 0],
            #                     [0, 1, 0, 0],
            #                     [-math.sin(-math.pi/9), 0, math.cos(-math.pi/9), 0]])
            #     self.set_position_orientation(self.right_forearm_handle, self.Rt_right_forearm, R_a)
            #     aux = 1

            # Show frame and exit with "ESC"
            cv2.imshow('Image', img)
            tecla = cv2.waitKey(5) & 0xFF
            if tecla == 27:
                break

        self.stop_simulation()
        self.arduino.close()
