import numpy as np
from MotionPlanning.kinematics.kinematics_utils import rot_z


class ConstTrajectory:
    def __init__(self):
        pass

    def get_position(self, t: float) -> np.ndarray:
        return np.array([0., 0., 0.])


class LegTrajectory:
    def __init__(self, leg_no, leg_pos, cycle_time):
        self.leg_no = leg_no
        self.leg_pos = leg_pos
        self.cycle_time = cycle_time

    def _pos_without_omega(self, t: float, tref: float,
                           v: np.ndarray) -> np.ndarray:
        p0 = 0.25*self.cycle_time*v
        return np.array([p0[0] - v[0]*tref,
                         p0[1] - v[1]*tref,
                         0. if t <= 0.5*self.cycle_time else
                         0.01*np.sqrt(1 - (4*tref /
                                           self.cycle_time - 1)**2)])

    def get_position(self, t: float,
                     v: np.ndarray, omega: float) -> np.ndarray:
        # Translate coordinate system
        v = np.array([v[0], -v[1]])

        if self.leg_no == 1 or self.leg_no == 3 or \
           self.leg_no == 5:
            # Three point gait - Shift trajectory of
            # legs 1, 3, 5 by half cycle
            if t <= 0.5*self.cycle_time:
                t += 0.5*self.cycle_time
            else:
                t -= 0.5*self.cycle_time

        if t > 0.5*self.cycle_time:
            tref = self.cycle_time - t
        else:
            tref = t

        if omega == 0.:
            # Handle special case of omega == 0
            point = self._pos_without_omega(t, tref, v)
        else:
            r_c = np.linalg.norm(v)/omega
            if np.linalg.norm(v) > 0.:
                vdir = v/np.linalg.norm(v)
            else:
                vdir = np.array([0., 0.])
            odir = np.array([-vdir[1], vdir[0]])
            center = odir*r_c

            rel_pos = center + self.leg_pos

            r = np.sqrt(rel_pos[0]**2 + rel_pos[1]**2)
            phi0 = np.arctan2(rel_pos[1], rel_pos[0])
            phi_m = phi0 - 0.25*self.cycle_time*omega

            point = np.array([r*np.cos(phi_m + omega*tref) - rel_pos[0],
                              r*np.sin(phi_m + omega*tref) - rel_pos[1],
                              0. if t <= 0.5*self.cycle_time else
                              0.01*np.sqrt(1 - (4*tref /
                                                self.cycle_time - 1)**2)])

        if self.leg_no <= 3:
            # Mirror coordinates for legs 1-3
            return np.array([-point[0], point[1], point[2]])
        else:
            return point


def adjust_leg_spacing(pos: np.ndarray, leg_spacing: float):
    pos[0] += (leg_spacing - 0.2) / 2
    return pos


def adjust_height(pos: np.ndarray, height: float):
    pos[2] -= height
    return pos
