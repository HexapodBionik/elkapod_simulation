from abc import ABC
import numpy as np
from ..movement.movement_leg_trajectory import LegTrajectory
from ..connection.connection_mockup import HardwareControllerConnection
from MotionPlanning.kinematics.kinematics_solvers import KinematicsSolver


class MovementProfile(ABC):
    def __init__(self):
        self._trajectories = []

    def generate_trajectories(self, step_length: float, movement_time: float, **kwargs) -> None:
        pass

    def execute_trajectories(self, t: float, connection: HardwareControllerConnection, kinematics_solver: KinematicsSolver):
        pass


class MechatronicGait(MovementProfile):
    def __init__(self):
        super().__init__()

    def generate_trajectories(self, step_length: float, movement_time: float, **kwargs) -> None:
        z_height = kwargs.get("z_height", -0.1)
        x_trans = kwargs.get("x_trans", 0.2)

        swing_height = kwargs.get("swing_height", 0.025)

        for i in range(6):
            p = np.array([x_trans, step_length/2 - i / 5 * step_length, z_height])
            trajectory = LegTrajectory(i, step_length, swing_height, movement_time, 1 / 6, p)
            trajectory.generate_symmetrical_trajectory()
            self._trajectories.append(trajectory)

    def execute_trajectories(self, t: float, connection: HardwareControllerConnection, kinematics_solver: KinematicsSolver):
        leg_list = [5, 4, 3, 2, 1, 0]
        for traj, leg in zip(self._trajectories, leg_list):
            p = traj.get_position(t)
            q = kinematics_solver.inverse(p)
            connection.set_leg_angles(leg, q)

        return np.array([traj.get_position(t) for traj in self._trajectories])




