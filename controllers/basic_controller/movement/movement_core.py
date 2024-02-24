import numpy as np

from ..movement.movement_profiles import MovementProfile, MechatronicGait
from ..connection.connection_mockup import HardwareControllerConnection
from ..movement.movement_errors import MovementCoreInitializationError
from MotionPlanning.kinematics.kinematics_solvers import KinematicsSolver


class MovementCore:
    def __init__(self, hardware_controller_conn: HardwareControllerConnection):
        self._base_height = 0
        self._step_length = 0.1

        self.connection = hardware_controller_conn
        self._trajectories_generator = None
        self.kinematics_solver = None

        self._last_action_time = 0

    def init(self, setup_dict: dict):
        diagram_variables = setup_dict.get("diagram_variables", None)

        if diagram_variables is None:
            raise MovementCoreInitializationError(
                "KinematicsSolver initialization has failed! "
                "'diagram_variables' key not found in setup dict!")

        m1 = np.array(diagram_variables.get("m1"))
        a1 = np.array(diagram_variables.get("a1"))
        a2 = np.array(diagram_variables.get("a2"))
        a3 = np.array(diagram_variables.get("a3"))

        if any(vector is None for vector in [m1, a1, a2, a3]):
            raise MovementCoreInitializationError(
                "Kinematics initialization has failed! 'diagram_variables' are corrupted! ")

        self.kinematics_solver = KinematicsSolver(m1, a1, a2, a3)

    def set_base_height(self):
        pass

    def generate_trajectories(self, velocity: float):
        movement_time = self._step_length / velocity
        self._last_action_time = movement_time

        self._trajectories_generator = MechatronicGait()
        self._trajectories_generator.generate_trajectories(self._step_length, movement_time)

    def execute_trajectories(self, t: float):
        self._trajectories_generator.execute_trajectories(t, self.connection, self.kinematics_solver)

    def is_action_finished(self, t: float):
        if t >= self._last_action_time:
            return True
        else:
            return False


