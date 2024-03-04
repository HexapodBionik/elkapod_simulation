from controller import Robot
from time import time
from trajectory import (
    get_position,
    adjust_yaw,
    adjust_pitch,
    adjust_roll,
    adjust_leg_spacing,
    adjust_height,
)
from MotionPlanning.kinematics.kinematics_solvers import KinematicsSolver

from PySide6.QtCore import (
    QObject,
    Signal,
    Slot,
    QCoreApplication,
)

import numpy as np
import math

# Create robot instance.
robot = Robot()

# Get the time step of the current world.
timestep = int(robot.getBasicTimeStep())

# Obtaining instances of the devices of the robot.
servo11 = robot.getDevice("Servo11")
servo12 = robot.getDevice("Servo12")
servo13 = robot.getDevice("Servo13")

servo21 = robot.getDevice("Servo21")
servo22 = robot.getDevice("Servo22")
servo23 = robot.getDevice("Servo23")

servo31 = robot.getDevice("Servo31")
servo32 = robot.getDevice("Servo32")
servo33 = robot.getDevice("Servo33")

servo41 = robot.getDevice("Servo41")
servo42 = robot.getDevice("Servo42")
servo43 = robot.getDevice("Servo43")

servo51 = robot.getDevice("Servo51")
servo52 = robot.getDevice("Servo52")
servo53 = robot.getDevice("Servo53")

servo61 = robot.getDevice("Servo61")
servo62 = robot.getDevice("Servo62")
servo63 = robot.getDevice("Servo63")


class Leg:
    def __init__(self, s1, s2, s3, offset, leg_name: str, sign):
        self._s1 = s1
        self._s2 = s2
        self._s3 = s3
        self._offset = offset
        self._leg_name = leg_name
        self._sign = sign

        self.normal_position = True

    def set_angles(self, q: np.ndarray) -> None:
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

    def __str__(self):
        return self._leg_name


leg1 = Leg(servo11, servo12, servo13, 0, "l1", -1)
leg2 = Leg(servo21, servo22, servo23, 0, "l2", -1)
leg3 = Leg(servo31, servo32, servo33, 0, "l3", -1)
leg4 = Leg(servo41, servo42, servo43, math.pi, "l4", 1)
leg5 = Leg(servo51, servo52, servo53, math.pi, "l5", 1)
leg6 = Leg(servo61, servo62, servo63, math.pi, "l6", 1)

legs = [leg1, leg2, leg3, leg4, leg5, leg6]

# Hexapod dimensions
mount_t = np.array([0, 0, 0.05])
t1 = np.array([0.063, 0, 0])
t2 = np.array([0.09, 0, 0])
t3 = np.array([0.23, 0, 0])

hexapod_kinematics_solver = KinematicsSolver(mount_t, t1, t2, t3)


class WebotsWorker(QObject):
    finished = Signal()
    stop = Signal()

    leg_spacing_signal = Signal(int)
    height_signal = Signal(int)
    vdir_signal = Signal(int)
    vval_signal = Signal(int)
    omega_signal = Signal(int)
    yaw_signal = Signal(int)
    pitch_signal = Signal(int)
    roll_signal = Signal(int)
    step_height_signal = Signal(int)

    @Slot(int)
    def update_leg_spacing(self, leg_spacing):
        self.leg_spacing = (leg_spacing/40)*0.2

    @Slot(int)
    def update_height(self, height):
        self.height = (height/100)*0.1

    @Slot(int)
    def update_vdir(self, vdir):
        self.vdir = ((vdir-90)/360)*2*np.pi

    @Slot(int)
    def update_vval(self, vval):
        self.vval = (vval/100)*0.03

    @Slot(int)
    def update_omega(self, omega):
        self.omega = -(omega*2*np.pi/3600)

    @Slot(int)
    def update_yaw(self, yaw):
        self.yaw = yaw*2*np.pi/3600

    @Slot(int)
    def update_pitch(self, pitch):
        self.pitch = pitch*2*np.pi/3600

    @Slot(int)
    def update_roll(self, roll):
        self.roll = roll*2*np.pi/3600

    @Slot(int)
    def update_step_height(self, step_height):
        self.step_height = (step_height/10000.)

    @Slot()
    def stop_worker(self):
        self.should_stop = True

    def run(self):
        self.should_stop = False
        self.stop.connect(self.stop_worker)

        self.leg_spacing = 0.6
        self.height = 0.1
        self.vdir = np.pi/2
        self.vval = 0.
        self.omega = 0.
        self.yaw = 0.
        self.pitch = 0.
        self.roll = 0.
        self.step_height = 0.

        self.leg_spacing_signal.connect(self.update_leg_spacing)
        self.height_signal.connect(self.update_height)
        self.vdir_signal.connect(self.update_vdir)
        self.vval_signal.connect(self.update_vval)
        self.omega_signal.connect(self.update_omega)
        self.yaw_signal.connect(self.update_yaw)
        self.pitch_signal.connect(self.update_pitch)
        self.roll_signal.connect(self.update_roll)
        self.step_height_signal.connect(self.update_step_height)

        # Homing
        pos0g0 = np.array([0.2, 0., -0.1])
        pos0g1 = np.array([0.2, 0., -0.1])
        qg0 = hexapod_kinematics_solver.inverse(pos0g0)
        qg1 = hexapod_kinematics_solver.inverse(pos0g1)
        for i in range(6):
            if not (i % 2):
                legs[i].set_angles(qg0)
            else:
                legs[i].set_angles(qg1)
        t0 = time()
        while time() - t0 < 1.:
            robot.step(timestep)

        # Main loop
        t0 = time()
        while robot.step(timestep) != -1:
            # Process events from Qt part
            QCoreApplication.processEvents()
            if self.should_stop:
                break

            curr_time = time()
            if curr_time - t0 < 6.:
                # Generate trajectories
                leg_positions = [
                        np.array([self.leg_spacing/2, 0.15, -self.height]),
                        np.array([self.leg_spacing/2, 0., -self.height]),
                        np.array([self.leg_spacing/2, -0.15, -self.height]),
                        np.array([-self.leg_spacing/2, 0.15, -self.height]),
                        np.array([-self.leg_spacing/2, 0., -self.height]),
                        np.array([-self.leg_spacing/2, -0.15, -self.height]),
                ]
                for leg_no, (leg_pos, leg) in \
                        enumerate(zip(leg_positions, legs), start=1):
                    pos = get_position(
                        curr_time - t0,     # timepoint
                        6.,                 # cycle duration
                        leg_no,             # leg number
                        leg_pos,            # leg position (relative to center
                                            #               of the corpus)
                        self.step_height,   # step_height
                        np.array([self.vval *           # velocity vector
                                  np.cos(self.vdir),
                                  self.vval *
                                  np.sin(self.vdir)]),
                        self.omega          # angular velocity
                    )
                    pos = adjust_leg_spacing(pos, self.leg_spacing)
                    pos = adjust_height(pos, self.height)
                    pos = adjust_yaw(pos, self.yaw, self.leg_spacing,
                                     leg_no, leg_pos)
                    pos = adjust_pitch(pos, self.pitch, leg_pos)
                    pos = adjust_roll(pos, self.roll, leg_no, leg_pos)
                    q = hexapod_kinematics_solver.inverse(pos)
                    leg.set_angles(q)
            else:
                t0 = curr_time

        # Tell Qt part that we have finished
        self.finished.emit()
