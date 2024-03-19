from time import time
import numpy as np

from PySide6.QtCore import (
    QObject,
    Signal,
    Slot,
    QCoreApplication,
)

from mega_lynx import MegaLynx
from trajectory import traj_shape
from gait import build_gait


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
    gait_signal = Signal(str)
    mode_signal = Signal(str)

    @Slot(int)
    def update_leg_spacing(self, leg_spacing):
        self.leg_spacing = (leg_spacing/40)*0.2

    @Slot(int)
    def update_height(self, height):
        self.height = (height/100)*0.1

    @Slot(int)
    def update_vdir(self, vdir):
        self.vdir = ((vdir+90)/360)*2*np.pi

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

    @Slot(int)
    def update_gait(self, gait):
        if self.mode == 'REGULAR':
            self.time_mappings = build_gait(gait, self.cycle_time, 6)
        else:
            self.time_mappings = build_gait(gait, self.cycle_time, 4)

    @Slot(int)
    def update_mode(self, mode):
        self.mode = mode

    @Slot()
    def stop_worker(self):
        self.should_stop = True

    def init_and_home(self):
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

        self.cycle_time = 6.
        self.time_mappings = build_gait('3POINT', self.cycle_time, 6)
        self.mode = 'REGULAR'

        self.leg_spacing_signal.connect(self.update_leg_spacing)
        self.height_signal.connect(self.update_height)
        self.vdir_signal.connect(self.update_vdir)
        self.vval_signal.connect(self.update_vval)
        self.omega_signal.connect(self.update_omega)
        self.yaw_signal.connect(self.update_yaw)
        self.pitch_signal.connect(self.update_pitch)
        self.roll_signal.connect(self.update_roll)
        self.step_height_signal.connect(self.update_step_height)
        self.gait_signal.connect(self.update_gait)
        self.mode_signal.connect(self.update_mode)

        self.robot = MegaLynx()

        # Homing
        for i in range(6):
            self.robot.legs[i].set_pose(np.array([0.2, 0., -0.1]))
        t0 = time()
        while time() - t0 < 1.:
            self.robot.simulation_step()

    def run(self):
        self.init_and_home()

        t0 = time()
        while self.robot.simulation_step() != -1:
            # Process events from Qt part
            QCoreApplication.processEvents()
            if self.should_stop:
                break

            # Update leg positions
            if self.mode == 'REGULAR':
                supportive_legs = [1, 2, 3, 4, 5, 6]
                leg_positions = [
                        np.array([self.leg_spacing/2, 0.15]),
                        np.array([self.leg_spacing/2, 0.]),
                        np.array([self.leg_spacing/2, -0.15]),
                        np.array([-self.leg_spacing/2, 0.15]),
                        np.array([-self.leg_spacing/2, 0.]),
                        np.array([-self.leg_spacing/2, -0.15]),
                ]
            else:
                supportive_legs = [2, 3, 5, 6]
                leg_positions = [
                        np.array([self.leg_spacing/2, 0.15]),
                        np.array([self.leg_spacing/2, -0.3]),
                        np.array([-self.leg_spacing/2, 0.15]),
                        np.array([-self.leg_spacing/2, -0.3]),
                ]
#             leg_positions = [
#                     np.array([self.leg_spacing/2, 0.15]),
#                     np.array([self.leg_spacing/2, 0.]),
#                     np.array([self.leg_spacing/2, -0.15]),
#                     np.array([-self.leg_spacing/2, 0.15]),
#                     np.array([-self.leg_spacing/2, 0.]),
#                     np.array([-self.leg_spacing/2, -0.15]),
#             ]
# 
#             if self.mode == 'TRANSPORTER':
#                 supportive_legs = [2, 3, 5, 6]
#             else:
#                 supportive_legs = [1, 2, 3, 4, 5, 6]
# 
#             leg_positions = [
#                 leg_positions[i-1]
#                 for i in supportive_legs
#             ]

            # Update trajectories
            leg_trajs = []
            for leg_no, leg_pos, time_mapping in \
                    zip(supportive_legs,
                        leg_positions,
                        self.time_mappings):
                shape = traj_shape(
                    leg_pos,                        # leg position
                    leg_no,                         # leg number
                    np.array([0., 0., 1.]),         # normal to plane
                    np.array([self.vval *           # linear
                              np.cos(self.vdir),
                              self.vval *
                              np.sin(self.vdir)]) * self.cycle_time,
                    self.omega * self.cycle_time,   # angular
                    self.step_height,               # step height
                    self.height,                    # height
                    0.,                             # horizontal_shift
                    self.yaw,                       # yaw
                    self.pitch,                     # pitch
                    self.roll,                      # roll
                    0.1                             # margin
                )

                leg_trajs.append(
                    lambda t, shape=shape, time_mapping=time_mapping:
                        shape(time_mapping(t))
                )

            if self.mode == 'TRANSPORTER':
                leg_trajs.insert(2, lambda t: np.array([0., -0.3, 0.]))
                leg_trajs.insert(0, lambda t: np.array([0., -0.3, 0.]))

            curr_time = time()
            if curr_time - t0 >= self.cycle_time:
                t0 = curr_time

            for leg, traj in zip(self.robot.legs, leg_trajs):
                leg.set_pose(traj(curr_time - t0))

        # Tell Qt part that we have finished
        self.finished.emit()
