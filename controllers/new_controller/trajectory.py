import numpy as np
from MotionPlanning.kinematics.kinematics_utils import rot_x, rot_y, rot_z


def _pos_without_omega(t: float, tref: float,
                       cycle_time: float, step_height: float,
                       v: np.ndarray) -> np.ndarray:
    """
    Helper function. To be used when angular velocity
    `omega` is zero.

    Generated trajectory is a fragment of straight line
    in x, y.
    """
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
    """
    Generate base trajectory for legs of
    a hexapod moving with given velocity `v` in
    any direction and angular velocity `omega`.

    Such trajectory is fragment of a circle in
    x, y unless angular velocity `omega` is zero.
    In such a case helper solver is used:
    `_pos_without_omega`.

    Three-Point Gait is assumed.
    """

    # Translate coordinate system
    # FIXME: Make this step unnecessary
    #        (update equations)
    v = np.array([v[0], -v[1]])

    # In Three-Point-Gait legs 1, 3, 5
    # should move in opposite direction
    # than legs 2, 4, 6 so we shift their
    # trajectories by 0.5*cycle_time
    if leg_no == 1 or leg_no == 3 or \
       leg_no == 5:
        # Three point gait - Shift trajectory of
        # legs 1, 3, 5 by half cycle
        if t <= 0.5*cycle_time:
            t += 0.5*cycle_time
        else:
            t -= 0.5*cycle_time

    # In second part of the cycle,
    # legs follow same trajectory
    # in x, y, but in opposite direction.
    #
    # t    - true timepoint
    # tref - fake timepoint which
    #        incrases during first
    #        part of the cycle and
    #        decrases during second
    #        part of the cycle
    if t > 0.5*cycle_time:
        tref = cycle_time - t
    else:
        tref = t

    if omega == 0.:
        # Handle special case of omega == 0
        # Trajectory in x, y is fragment of a line
        point = _pos_without_omega(t, tref, cycle_time, step_height, v)
    else:
        # Trajectory in x, y is fragment of a circle

        # r_c - fragment of the circle followed by hexapod
        r_c = np.linalg.norm(v)/omega

        # vdir - unit vector of direction of the velocity `v`
        if np.linalg.norm(v) > 0.:
            vdir = v/np.linalg.norm(v)
        else:
            vdir = np.array([0., 0.])

        # center - center of the circle followed by hexapod
        odir = np.array([-vdir[1], vdir[0]])
        center = odir*r_c

        # position of the leg relative to the center of
        # the circle followed by hexapod
        rel_pos = center + leg_pos[:2]

        # r - Radius of a circle with same center but
        #     crossing end of the current leg in xy plane.
        r = np.sqrt(rel_pos[0]**2 + rel_pos[1]**2)

        # phi0 - position of the end of the current leg
        #        on such circle
        phi0 = np.arctan2(rel_pos[1], rel_pos[0])

        # phi_m - In timepoint 0 our leg has position phi_m
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
