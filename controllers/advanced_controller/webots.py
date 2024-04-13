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
    sl_signal = Signal(list)
    corpus_position_signal = Signal(np.ndarray)

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

    @Slot(str)
    def update_gait(self, gait):
        self.gait = gait

        # Rebuild gait
        self.time_mappings = build_gait(self.gait, self.cycle_time,
                                        self.supportive_legs)

    @Slot(list)
    def update_sl(self, supportive_legs):
        self.supportive_legs = supportive_legs

        # Rebuild gait
        self.time_mappings = build_gait(self.gait, self.cycle_time,
                                        self.supportive_legs)

    @Slot(np.ndarray)
    def update_corpus_position(self, position):
        self.corpus_position = position

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
        self.corpus_position = np.array([0., 0.])

        self.cycle_time = 6.
        self.supportive_legs = [True for _ in range(6)]
        self.gait = '3POINT'
        self.time_mappings = build_gait(self.gait, self.cycle_time,
                                        self.supportive_legs)

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
        self.sl_signal.connect(self.update_sl)
        self.corpus_position_signal.connect(self.update_corpus_position)

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
            leg_positions = [
                    np.array([self.leg_spacing/2, 0.15]),
                    np.array([self.leg_spacing/2, 0.]),
                    np.array([self.leg_spacing/2, -0.15]),
                    np.array([-self.leg_spacing/2, 0.15]),
                    np.array([-self.leg_spacing/2, 0.]),
                    np.array([-self.leg_spacing/2, -0.15]),
            ]

            # Update trajectories
            leg_trajs = []
            for leg_no, (leg_pos, time_mapping) in \
                    enumerate(
                        zip(leg_positions,
                            self.time_mappings),
                        start=1):
                if time_mapping is None:
                    leg_trajs.append(lambda t: np.array([0.32, 0., 0.]))
                    continue

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
                    self.corpus_position,           # corpus position
                    self.yaw,                       # yaw
                    self.pitch,                     # pitch
                    self.roll,                      # roll
                    0.1                             # margin
                )

                leg_trajs.append(
                    lambda t, shape=shape, time_mapping=time_mapping:
                        shape(time_mapping(t))
                )

            curr_time = time()
            if curr_time - t0 >= self.cycle_time:
                t0 = curr_time

            for leg, traj in zip(self.robot.legs, leg_trajs):
                leg.set_pose(traj(curr_time - t0))

        # Tell Qt part that we have finished
        self.finished.emit()
