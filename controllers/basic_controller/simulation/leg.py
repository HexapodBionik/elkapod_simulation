from controller.motor import Motor
import numpy as np


class Leg:
    def __init__(self, servo1: Motor, servo2: Motor, servo3: Motor, offset, leg_name: str, sign):
        self._s1 = servo1
        self._s2 = servo2
        self._s3 = servo3

        self._offset = offset
        self._leg_name = leg_name
        self._sign = sign

        self.normal_position = True

    def __str__(self):
        return self._leg_name

    def set_angles(self, q: np.ndarray):
        q1 = np.deg2rad(q[0])
        q2 = np.deg2rad(q[1])
        q3 = np.deg2rad(q[2])

        self._s1.setPosition(self._offset + q1 * self._sign)
        self._s2.setPosition(q2)
        self._s3.setPosition(q3)

        if q1 == 0 and q3 == 0:
            self.normal_position = True
        else:
            self.normal_position = False


