import cv2
import math
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

        # Reference of the elements
        _, self.cam_handle = self.reference_handle('NAO_vision1')

        _, self.RShoulderPitch3 = self.reference_handle('RShoulderPitch3')

        # Start the elements and wait a second to fill the buffer
        # orientation of the right arm
        self.orientation_RShoulderPitch3 = self.opmode_first_call(self.RShoulderPitch3)

        # resolution and image of the cam
        _, _ = self.opmode_first_call_cam(self.cam_handle)

        # Init communication arduino
        if self.com_arduino:
            FunctionsArduino.__init__(self, COM)

    def nothing(self, *arg):
        pass

    def run(self):
        cont = 0
        # help graph
        icol = (0, 0, 0)
        cv2.namedWindow('Angles')
        cv2.createTrackbar('alpha', 'Angles', icol[0], 360, self.nothing)
        cv2.createTrackbar('beta', 'Angles', icol[1], 360, self.nothing)
        cv2.createTrackbar('gamma', 'Angles', icol[2], 360, self.nothing)

        while 1:
            # Save frame of the camera, rotate it and convert it to BGR
            img = self.get_image(self.cam_handle)

            # Real time orientation handle
            self.orientation_RShoulderPitch3 = self.get_orientation(self.RShoulderPitch3)

            if self.orientation_RShoulderPitch3[0] > 0:
                alpha = self.orientation_RShoulderPitch3[0]
            else:
                alpha = 2 * math.pi + self.orientation_RShoulderPitch3[0]

            if self.orientation_RShoulderPitch3[1] > 0:
                beta = self.orientation_RShoulderPitch3[1]
            else:
                beta = 2 * math.pi + self.orientation_RShoulderPitch3[1]

            if self.orientation_RShoulderPitch3[2] > 0:
                gamma = self.orientation_RShoulderPitch3[2]
            else:
                gamma = 2 * math.pi + self.orientation_RShoulderPitch3[2]

            cv2.setTrackbarPos('alpha', 'Angles', int(alpha * 180 / math.pi))
            cv2.setTrackbarPos('beta', 'Angles', int(beta * 180 / math.pi))
            cv2.setTrackbarPos('gamma', 'Angles', int(gamma * 180 / math.pi))

            # if self.com_arduino:
            #     value = self.get_gyroscope()
            #     print(value)
            # else:
            #     value = int(alpha * 180 / math.pi)
            #
            # new_alpha = value
            # if new_alpha < 180:
            #     new_alpha = new_alpha
            # else:
            #     new_alpha = new_alpha - 360
            #
            # self.orientation_RShoulderPitch3[0] = new_alpha * math.pi / 180

            # self.orientation_RShoulderPitch3[0] = -92 * math.pi / 180
            #
            # self.set_orientation(self.RShoulderPitch3, self.orientation_RShoulderPitch3)

            # Show frame and exit with "ESC"
            cv2.imshow('Image', img)
            tecla = cv2.waitKey(25) & 0xFF
            if tecla == 27:
                break

        self.stop_simulation()
        if self.com_arduino:
            self.arduino.close()
