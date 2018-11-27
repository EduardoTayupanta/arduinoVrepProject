import math
import numpy as np


class MatrixRt:
    # Calculates Rotation Matrix given euler angles.
    def eulerAnglesToRotationMatrix(self, orientation):
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
        return R

    def turn_roll(self, R, angle):
        R_x = np.array([[1, 0, 0],
                        [0, math.cos(angle), -math.sin(angle)],
                        [0, math.sin(angle), math.cos(angle)]])
        return np.dot(R_x, R)

    def turn_pitch(self, R, angle):
        R_y = np.array([[math.cos(angle), 0, math.sin(angle)],
                        [0, 1, 0],
                        [-math.sin(angle), 0, math.cos(angle)]])
        return np.dot(R_y, R)

    def turn_yaw(self, R, angle):
        R_z = np.array([[math.cos(angle), -math.sin(angle), 0],
                        [math.sin(angle), math.cos(angle), 0],
                        [0, 0, 1]])
        return np.dot(R_z, R)

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
