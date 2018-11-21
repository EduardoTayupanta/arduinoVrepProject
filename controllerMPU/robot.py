import cv2
import time
from threading import Thread

from FunctionsVRep import FunctionsVRep
from FunctionsArduino import FunctionsArduino


class Robot(Thread, FunctionsVRep, FunctionsArduino):
    def __init__(self, clientID, arduino, COM):
        ''' Constructor. '''
        Thread.__init__(self)
        self.com_arduino = arduino

        # Init Simulation
        FunctionsVRep.__init__(self, clientID)
        self.start_simultion()

        # RShoulder
        _, self.RShoulderPitch3_handle = self.reference_handle('RShoulderPitch3')
        _, self.RShoulderRoll3_handle = self.reference_handle('RShoulderRoll3')
        # RElbow
        _, self.RElbowRoll3_handle = self.reference_handle('RElbowRoll3')
        _, self.RElbowYaw3_handle = self.reference_handle('RElbowYaw3')

        # LShoulder
        _, self.LShoulderPitch3_handle = self.reference_handle('LShoulderPitch3')
        _, self.LShoulderRoll3_handle = self.reference_handle('LShoulderRoll3')
        # LElbow
        _, self.LElbowRoll3_handle = self.reference_handle('LElbowRoll3')
        _, self.LElbowYaw3_handle = self.reference_handle('LElbowYaw3')

        # Start the elements and wait a second to fill the buffer
        # position of the right arm
        # RShoulder
        self.RShoulderPitch3_position = None
        self.RShoulderRoll3_position = None
        _ = self.opmode_first_call(self.RShoulderPitch3_handle)
        _ = self.opmode_first_call(self.RShoulderRoll3_handle)
        # RElbow
        self.RElbowRoll3_position = None
        self.RElbowYaw3_position = None
        _ = self.opmode_first_call(self.RElbowRoll3_handle)
        _ = self.opmode_first_call(self.RElbowYaw3_handle)

        # position of the left arm
        # LShoulder
        self.LShoulderPitch3_position = None
        self.LShoulderRoll3_position = None
        _ = self.opmode_first_call(self.LShoulderPitch3_handle)
        _ = self.opmode_first_call(self.LShoulderRoll3_handle)
        # LElbow
        self.LElbowRoll3_position = None
        self.LElbowYaw3_position = None
        _ = self.opmode_first_call(self.LElbowRoll3_handle)
        _ = self.opmode_first_call(self.LElbowYaw3_handle)

        # Init communication arduino
        if self.com_arduino:
            FunctionsArduino.__init__(self, COM)

    def nothing(self, *arg):
        pass

    def run(self):
        # init graph
        cv2.namedWindow('RightArm')
        cv2.resizeWindow('RightArm', 256, 256)

        self.RShoulderPitch3_position = self.get_position(self.RShoulderPitch3_handle)
        self.RShoulderRoll3_position = self.get_position(self.RShoulderRoll3_handle)
        self.RElbowRoll3_position = self.get_position(self.RElbowRoll3_handle)
        self.RElbowYaw3_position = self.get_position(self.RElbowYaw3_handle)

        cv2.createTrackbar('RSPitch', 'RightArm', self.RShoulderPitch3_position, 180, self.nothing)
        cv2.createTrackbar('RSRoll', 'RightArm', self.RShoulderRoll3_position, 180, self.nothing)
        cv2.createTrackbar('REIRoll', 'RightArm', self.RElbowRoll3_position, 180, self.nothing)
        cv2.createTrackbar('REIYaw', 'RightArm', self.RElbowYaw3_position, 180, self.nothing)

        cv2.namedWindow('LightArm')
        cv2.resizeWindow('LightArm', 256, 256)

        self.LShoulderPitch3_position = self.get_position(self.LShoulderPitch3_handle)
        self.LShoulderRoll3_position = self.get_position(self.LShoulderRoll3_handle)
        self.LElbowRoll3_position = self.get_position(self.LElbowRoll3_handle)
        self.LElbowYaw3_position = self.get_position(self.LElbowYaw3_handle)

        cv2.createTrackbar('LSPitch', 'LightArm', self.LShoulderPitch3_position, 180, self.nothing)
        cv2.createTrackbar('LSRoll', 'LightArm', self.LShoulderRoll3_position, 180, self.nothing)
        cv2.createTrackbar('LEIRoll', 'LightArm', self.LElbowRoll3_position, 180, self.nothing)
        cv2.createTrackbar('LEIYaw', 'LightArm', self.LElbowYaw3_position, 180, self.nothing)

        while 1:
            # Real time position handle
            # RightArm
            self.set_joint_target_position(self.RShoulderPitch3_handle,
                                           cv2.getTrackbarPos('RSPitch', 'RightArm'))
            self.set_joint_target_position(self.RShoulderRoll3_handle,
                                           cv2.getTrackbarPos('RSRoll', 'RightArm'))
            self.set_joint_target_position(self.RElbowRoll3_handle,
                                           cv2.getTrackbarPos('REIRoll', 'RightArm'))
            self.set_joint_target_position(self.RElbowYaw3_handle,
                                           cv2.getTrackbarPos('REIYaw', 'RightArm'))

            # LightArm
            self.set_joint_target_position(self.LShoulderPitch3_handle,
                                           cv2.getTrackbarPos('LSPitch', 'LightArm'))
            self.set_joint_target_position(self.LShoulderRoll3_handle,
                                           cv2.getTrackbarPos('LSRoll', 'LightArm'))
            self.set_joint_target_position(self.LElbowRoll3_handle,
                                           cv2.getTrackbarPos('LEIRoll', 'LightArm'))
            self.set_joint_target_position(self.LElbowYaw3_handle,
                                           cv2.getTrackbarPos('LEIYaw', 'LightArm'))

            time.sleep(0.2)

            # Show frame and exit with "ESC"
            tecla = cv2.waitKey(5) & 0xFF
            if tecla == 27:
                break

        time.sleep(1)
        self.stop_simulation()
        if self.com_arduino:
            self.arduino.close()
