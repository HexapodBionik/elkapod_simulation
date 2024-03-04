import numpy as np
from MotionPlanning.kinematics.kinematics_utils import rot_x, rot_y, rot_z


def _pos_without_omega(t: float, tref: float,
                       cycle_time: float, step_height: float,
                       v: np.ndarray) -> np.ndarray:
    p0 = 0.25*cycle_time*v
    return np.array([p0[0] - v[0]*tref,
                     p0[1] - v[1]*tref,
                     0. if t <= 0.5*cycle_time else
                     step_height*np.sqrt(1 - (4*tref /
                                         cycle_time - 1)**2)])


def get_position(t: float, cycle_time: float,
                 leg_no: int, leg_pos: np.ndarray,
                 step_height: float, v: np.ndarray,
                 omega: float) -> np.ndarray:
    # Translate coordinate system
    v = np.array([v[0], -v[1]])

    if leg_no == 1 or leg_no == 3 or \
       leg_no == 5:
        # Three point gait - Shift trajectory of
        # legs 1, 3, 5 by half cycle
        if t <= 0.5*cycle_time:
            t += 0.5*cycle_time
        else:
            t -= 0.5*cycle_time

    if t > 0.5*cycle_time:
        tref = cycle_time - t
    else:
        tref = t

    if omega == 0.:
        # Handle special case of omega == 0
        point = _pos_without_omega(t, tref, cycle_time, step_height, v)
    else:
        r_c = np.linalg.norm(v)/omega
        if np.linalg.norm(v) > 0.:
            vdir = v/np.linalg.norm(v)
        else:
            vdir = np.array([0., 0.])
        odir = np.array([-vdir[1], vdir[0]])
        center = odir*r_c

        rel_pos = center + leg_pos[:2]

        r = np.sqrt(rel_pos[0]**2 + rel_pos[1]**2)
        phi0 = np.arctan2(rel_pos[1], rel_pos[0])
        phi_m = phi0 - 0.25*cycle_time*omega

        point = np.array([r*np.cos(phi_m + omega*tref) - rel_pos[0],
                          r*np.sin(phi_m + omega*tref) - rel_pos[1],
                          0. if t <= 0.5*cycle_time else
                          step_height*np.sqrt(1 - (4*tref /
                                              cycle_time - 1)**2)])

    if leg_no <= 3:
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


def adjust_yaw(pos: np.ndarray, yaw: float,
               leg_spacing: float,
               leg_no: int, leg_pos: np.ndarray):
    if leg_no <= 3:
        pos[2] += 0.5*leg_spacing*np.tan(yaw)
        pos = rot_y(-yaw).dot(pos)
    else:
        pos[2] -= 0.5*leg_spacing*np.tan(yaw)
        pos = rot_y(yaw).dot(pos)

    pos[0] += 0.1*(1. - np.cos(yaw))

    return pos


def adjust_pitch(pos: np.ndarray, pitch: float, leg_pos: np.ndarray):
    pos[2] -= leg_pos[2]
    pos = rot_x(-pitch).dot(pos)
    pos[2] += leg_pos[2]
    delta_z = leg_pos[1]*np.tan(pitch) - \
        (leg_pos[2] + leg_pos[1]*np.tan(pitch))*(np.sin(pitch)**2)
    delta_y = (leg_pos[2] + leg_pos[1]*np.tan(pitch)) * \
        np.sin(pitch)*np.cos(pitch)

    pos[1] += delta_y
    pos[2] += delta_z

    return pos


def adjust_roll(pos: np.ndarray, roll: float,
                leg_no: int, leg_pos: np.ndarray):
    if leg_no <= 3:
        pos[0] -= leg_pos[1]*np.sin(roll)
        pos = rot_z(-roll).dot(pos)
    else:
        pos[0] += leg_pos[1]*np.sin(roll)
        pos = rot_z(roll).dot(pos)

    return pos
