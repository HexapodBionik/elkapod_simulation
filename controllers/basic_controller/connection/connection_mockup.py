import numpy as np
from basic_controller.simulation.simulation_setup import *


class HardwareControllerConnection:
    def __init__(self):
        self._leg_dict = {
            0: leg1, 1: leg2, 2: leg3, 3: leg4, 4: leg5, 5: leg6
        }

    def set_leg_angles(self, leg_nb: int, q: np.ndarray):
        leg = self._leg_dict.get(leg_nb, None)
        if leg is None:
            raise Exception("Invalid leg number!")
        elif leg is not None:
            leg.set_angles(q)
