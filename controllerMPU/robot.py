import cv2
import math
import time
from threading import Thread

from FunctionsVRep import FunctionsVRep
from FunctionsArduino import FunctionsArduino
from MatrixRt import MatrixRt


class Robot(Thread, FunctionsVRep, FunctionsArduino, MatrixRt):
    def __init__(self, clientID, arduino, COM, baud, graphic):
        ''' Constructor. '''
        Thread.__init__(self)
        self.com_arduino = arduino
        self.graphic = graphic

        # Init Simulation
        FunctionsVRep.__init__(self, clientID)
        self.start_simultion()

        _, self.HeadYaw_handle = self.reference_handle('HeadYaw')
        _, self.HeadPitch_handle = self.reference_handle('HeadPitch')

        _, self.NAO_vision1_handle = self.reference_handle('NAO_vision1')

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

        # RHip
        _, self.RHipPitch3_handle = self.reference_handle('RHipPitch3')
        _, self.RHipRoll3_handle = self.reference_handle('RHipRoll3')
        # RKnee
        _, self.RKneePitch3_handle = self.reference_handle('RKneePitch3')

        # LHip
        _, self.LHipPitch3_handle = self.reference_handle('LHipPitch3')
        _, self.LHipRoll3_handle = self.reference_handle('LHipRoll3')
        # LKnee
        _, self.LKneePitch3_handle = self.reference_handle('LKneePitch3')

        # Start the elements and wait a second to fill the buffer
        _, _ = self.opmode_first_call_image(self.NAO_vision1_handle)

        self.HeadYaw_position = None
        self.HeadPitch_position = None
        _ = self.opmode_first_call(self.HeadYaw_handle)
        _ = self.opmode_first_call(self.HeadPitch_handle)
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

        # position of the right leg
        # RHip
        self.RHipPitch3_position = None
        self.RHipRoll3_position = None
        _ = self.opmode_first_call(self.RHipPitch3_handle)
        _ = self.opmode_first_call(self.RHipRoll3_handle)
        # RKnee
        self.RKneePitch3_position = None
        _ = self.opmode_first_call(self.RKneePitch3_handle)

        # position of the left leg
        # LHip
        self.LHipPitch3_position = None
        self.LHipRoll3_position = None
        _ = self.opmode_first_call(self.LHipPitch3_handle)
        _ = self.opmode_first_call(self.LHipRoll3_handle)
        # LKnee
        self.LKneePitch3_position = None
        _ = self.opmode_first_call(self.LKneePitch3_handle)

        # Init communication arduino
        if self.com_arduino:
            FunctionsArduino.__init__(self, COM, baud)
            self.cont = 0
            self.flag = True
            self.dictionary_0 = {
                'num_mpu': 0,
                'roll': 0,
                'pitch': 0,
                'yaw': 0
            }
            self.dictionary_1 = {
                'num_mpu': 0,
                'roll': 0,
                'pitch': 0,
                'yaw': 0
            }
            self.dictionary_2 = {
                'num_mpu': 0,
                'roll': 0,
                'pitch': 0,
                'yaw': 0
            }
            self.dictionary_3 = {
                'num_mpu': 0,
                'roll': 0,
                'pitch': 0,
                'yaw': 0
            }

    def nothing(self, *arg):
        pass

    def run(self):
        # init graph
        cv2.namedWindow('Head')
        cv2.resizeWindow('Head', 256, 256)

        self.HeadYaw_position = self.get_position(self.HeadYaw_handle)
        self.HeadPitch_position = self.get_position(self.HeadPitch_handle)

        cv2.createTrackbar('HeadYaw', 'Head', self.HeadYaw_position, 180, self.nothing)
        cv2.createTrackbar('HeadPitch', 'Head', self.HeadPitch_position, 180, self.nothing)

        R_r = None
        R_l = None

        if self.graphic[0] == 1:
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

        if self.graphic[1] == 1:
            cv2.namedWindow('LeftArm')
            cv2.resizeWindow('LeftArm', 256, 256)

            self.LShoulderPitch3_position = self.get_position(self.LShoulderPitch3_handle)
            self.LShoulderRoll3_position = self.get_position(self.LShoulderRoll3_handle)
            self.LElbowRoll3_position = self.get_position(self.LElbowRoll3_handle)
            self.LElbowYaw3_position = self.get_position(self.LElbowYaw3_handle)

            cv2.createTrackbar('LSPitch', 'LeftArm', self.LShoulderPitch3_position, 180, self.nothing)
            cv2.createTrackbar('LSRoll', 'LeftArm', self.LShoulderRoll3_position, 180, self.nothing)
            cv2.createTrackbar('LEIRoll', 'LeftArm', self.LElbowRoll3_position, 180, self.nothing)
            cv2.createTrackbar('LEIYaw', 'LeftArm', self.LElbowYaw3_position, 180, self.nothing)

        if self.graphic[2] == 1:
            cv2.namedWindow('RightLeg')
            cv2.resizeWindow('RightLeg', 256, 256)

            self.RHipPitch3_position = self.get_position(self.RHipPitch3_handle)
            self.RHipRoll3_position = self.get_position(self.RHipRoll3_handle)
            self.RKneePitch3_position = self.get_position(self.RKneePitch3_handle)

            cv2.createTrackbar('RHPitch', 'RightLeg', self.RHipPitch3_position, 180, self.nothing)
            cv2.createTrackbar('RHRoll', 'RightLeg', self.RHipRoll3_position, 180, self.nothing)
            cv2.createTrackbar('RKneePitch', 'RightLeg', self.RKneePitch3_position, 180, self.nothing)

        if self.graphic[3] == 1:
            cv2.namedWindow('LeftLeg')
            cv2.resizeWindow('LeftLeg', 256, 256)

            self.LHipPitch3_position = self.get_position(self.LHipPitch3_handle)
            self.LHipRoll3_position = self.get_position(self.LHipRoll3_handle)
            self.LKneePitch3_position = self.get_position(self.LKneePitch3_handle)

            cv2.createTrackbar('LHPitch', 'LeftLeg', self.LHipPitch3_position, 180, self.nothing)
            cv2.createTrackbar('LHRoll', 'LeftLeg', self.LHipRoll3_position, 180, self.nothing)
            cv2.createTrackbar('LKneePitch', 'LeftLeg', self.LKneePitch3_position, 180, self.nothing)

        while 1:
            # Real time position handle
            if not self.com_arduino:
                if self.graphic[0] == 1:
                    # RightArm
                    self.set_joint_target_position(self.RShoulderPitch3_handle,
                                                   cv2.getTrackbarPos('RSPitch', 'RightArm'))
                    self.set_joint_target_position(self.RShoulderRoll3_handle,
                                                   cv2.getTrackbarPos('RSRoll', 'RightArm'))
                    self.set_joint_target_position(self.RElbowRoll3_handle,
                                                   cv2.getTrackbarPos('REIRoll', 'RightArm'))
                    self.set_joint_target_position(self.RElbowYaw3_handle,
                                                   cv2.getTrackbarPos('REIYaw', 'RightArm'))

                if self.graphic[1] == 1:
                    # LeftArm
                    self.set_joint_target_position(self.LShoulderPitch3_handle,
                                                   cv2.getTrackbarPos('LSPitch', 'LeftArm'))
                    self.set_joint_target_position(self.LShoulderRoll3_handle,
                                                   cv2.getTrackbarPos('LSRoll', 'LeftArm'))
                    self.set_joint_target_position(self.LElbowRoll3_handle,
                                                   cv2.getTrackbarPos('LEIRoll', 'LeftArm'))
                    self.set_joint_target_position(self.LElbowYaw3_handle,
                                                   cv2.getTrackbarPos('LEIYaw', 'LeftArm'))

                if self.graphic[2] == 1:
                    # RightLeg
                    self.set_joint_target_position(self.RHipPitch3_handle,
                                                   cv2.getTrackbarPos('RHPitch', 'RightLeg'))
                    self.set_joint_target_position(self.RHipRoll3_handle,
                                                   cv2.getTrackbarPos('RHRoll', 'RightLeg'))
                    self.set_joint_target_position(self.RKneePitch3_handle,
                                                   cv2.getTrackbarPos('RKneePitch', 'RightLeg'))

                if self.graphic[3] == 1:
                    # LeftLeg
                    self.set_joint_target_position(self.LHipPitch3_handle,
                                                   cv2.getTrackbarPos('LHPitch', 'LeftLeg'))
                    self.set_joint_target_position(self.LHipRoll3_handle,
                                                   cv2.getTrackbarPos('LHRoll', 'LeftLeg'))
                    self.set_joint_target_position(self.LKneePitch3_handle,
                                                   cv2.getTrackbarPos('LKneePitch', 'LeftLeg'))

            else:
                self.set_joint_target_position(self.HeadYaw_handle,
                                               cv2.getTrackbarPos('HeadYaw', 'Head'))
                self.set_joint_target_position(self.HeadPitch_handle,
                                               cv2.getTrackbarPos('HeadPitch', 'Head'))
                self.get_gyroscope()
                dictionary = self.get_dictionary()
                if self.cont >= 50:
                    if dictionary['num_mpu'] == 0.0:
                        # print(dictionary)
                        # RightArm
                        R = self.eulerAnglesToRotationMatrix(
                            [dictionary['roll'] * math.pi / 180, dictionary['pitch'] * math.pi / 180,
                             dictionary['yaw'] * math.pi / 180])
                        R_r = R
                        angles = self.rotationMatrixToEulerAngles(R)

                        pitch = (math.degrees(angles[0]) +
                                 math.degrees(angles[1]) +
                                 math.degrees(angles[2])) / math.degrees(angles[0])
                        try:
                            roll = (math.degrees(angles[0]) +
                                    math.degrees(angles[1]) +
                                    math.degrees(angles[2])) / math.degrees(angles[1])
                        except ZeroDivisionError as error:
                            print(error)
                        yaw = (math.degrees(angles[0]) +
                               math.degrees(angles[1]) +
                               math.degrees(angles[2])) / math.degrees(angles[2])

                        if yaw >= 180:
                            yaw = 180

                        # print('pitch ', pitch)
                        # print('roll ', roll)
                        # print('yaw ', yaw)

                        if abs(pitch - roll) < 10.5:
                            self.dictionary_0['pitch'] = 180
                            self.dictionary_0['roll'] = -257.14 * yaw + 257.14
                        else:
                            self.dictionary_0['pitch'] = 10.45 * pitch + 199.23
                            self.dictionary_0['roll'] = 90

                        # R_pitch = self.turn_pitch(R, -math.pi / 2)
                        # angles_pitch = self.rotationMatrixToEulerAngles(R_pitch)
                        #
                        # new_pitch = 90 - angles_pitch[1] * 180 / math.pi
                        #
                        # self.dictionary_0['pitch'] = new_pitch
                        # self.dictionary_0['roll'] = 90 + dictionary['roll']

                        cv2.setTrackbarPos('RSPitch', 'RightArm', int(self.dictionary_0['pitch']))
                        cv2.setTrackbarPos('RSRoll', 'RightArm', int(self.dictionary_0['roll']))

                        self.set_joint_target_position(self.RShoulderPitch3_handle, self.dictionary_0['pitch'])
                        self.set_joint_target_position(self.RShoulderRoll3_handle, self.dictionary_0['roll'])

                    elif dictionary['num_mpu'] == 0.01:
                        # print(dictionary)
                        #     self.dictionary_1['yaw'] = 90 - dictionary['pitch']
                        self.dictionary_1['roll'] = -1.6 * dictionary['roll'] + 107.8
                        #
                        #     cv2.setTrackbarPos('REIYaw', 'RightArm', int(self.dictionary_1['yaw']))
                        cv2.setTrackbarPos('REIRoll', 'RightArm', int(self.dictionary_1['roll']))
                        #
                        #     self.set_joint_target_position(self.RElbowYaw3_handle, self.dictionary_1['yaw'])
                        self.set_joint_target_position(self.RElbowRoll3_handle, self.dictionary_1['roll'])
                    #
                    elif dictionary['num_mpu'] == 0.02:
                        # LeftArm
                        R = self.eulerAnglesToRotationMatrix(
                            [dictionary['roll'] * math.pi / 180, dictionary['pitch'] * math.pi / 180,
                             dictionary['yaw'] * math.pi / 180])
                        R_l = R
                        angles = self.rotationMatrixToEulerAngles(R)

                        pitch = (math.degrees(angles[0]) +
                                 math.degrees(angles[1]) +
                                 math.degrees(angles[2])) / math.degrees(angles[0])
                        roll = (math.degrees(angles[0]) +
                                math.degrees(angles[1]) +
                                math.degrees(angles[2])) / math.degrees(angles[1])
                        yaw = (math.degrees(angles[0]) +
                               math.degrees(angles[1]) +
                               math.degrees(angles[2])) / math.degrees(angles[2])

                        # print('pitch ', pitch)
                        # print('roll ', roll)
                        # print('yaw ', yaw)

                        if pitch - roll < 0:
                            self.dictionary_2['pitch'] = 180
                            self.dictionary_2['roll'] = 32.03 * pitch + 66.3
                        else:
                            self.dictionary_2['pitch'] = -200 * roll + 540
                            self.dictionary_2['roll'] = 90

                        # R_pitch = self.turn_pitch(R, -math.pi / 2)
                        # angles_pitch = self.rotationMatrixToEulerAngles(R_pitch)
                        #
                        # new_pitch = 90 - angles_pitch[1] * 180 / math.pi
                        #
                        # self.dictionary_2['pitch'] = new_pitch
                        # self.dictionary_2['roll'] = 90 - dictionary['roll']

                        cv2.setTrackbarPos('LSPitch', 'LeftArm', int(self.dictionary_2['pitch']))
                        cv2.setTrackbarPos('LSRoll', 'LeftArm', int(self.dictionary_2['roll']))

                        self.set_joint_target_position(self.LShoulderPitch3_handle, self.dictionary_2['pitch'])
                        self.set_joint_target_position(self.LShoulderRoll3_handle, self.dictionary_2['roll'])

                    elif dictionary['num_mpu'] == 0.03:
                        # print(dictionary)
                        #     self.dictionary_3['yaw'] = 90 - dictionary['pitch']
                        self.dictionary_3['roll'] = 2.08 * dictionary['roll'] + 120.92

                        #     cv2.setTrackbarPos('LEIYaw', 'LeftArm', int(self.dictionary_3['yaw']))
                        cv2.setTrackbarPos('LEIRoll', 'LeftArm', int(self.dictionary_3['roll']))

                        #     self.set_joint_target_position(self.LElbowYaw3_handle, self.dictionary_3['yaw'])
                        self.set_joint_target_position(self.LElbowRoll3_handle, self.dictionary_3['roll'])

                    self.flag = False
                if self.flag is True:
                    self.cont += 1
                    self.set_joint_target_position(self.RElbowYaw3_handle, 180)
                    self.set_joint_target_position(self.LElbowYaw3_handle, 0)

            # time.sleep(0.2)

            # img = self.get_image(self.NAO_vision1_handle)
            # cv2.imshow('Image', img)
            # Show frame and exit with "ESC"
            tecla = cv2.waitKey(1) & 0xFF
            if tecla == 27:
                break

        self.stop_simulation()
        time.sleep(1)
        self.stop_simulation()
        if self.com_arduino:
            self.arduino.close()
            time.sleep(1)
            self.arduino.close()
            self.arduino.close()
        print('Program ended')
