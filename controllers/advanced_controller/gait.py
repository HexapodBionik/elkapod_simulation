from typing import Callable
import numpy as np


def do_shift(t, T, t0, div):
    assert 0. <= t and t <= T
    assert 0. <= t0 and t0 < T

    t += t0
    if t > T:
        t -= T

    if t < T*div:
        return (2.*t/T)*0.5/div
    else:
        t -= T*div
        return 1. + (2.*t/T)*0.5/(1-div)


def traj_shift(T: float, t0: float, div: float) -> Callable[[float], float]:
    return (lambda t: do_shift(t, T, t0, div))


def _mod_one(x: float) -> float:
    """! Continous modulo one
    @param x: Any number
    @return: Number in range [0, 1)
    """
    return x - np.floor(x)


def build_gait(type: str,
               T: float,
               legs_cnt: int) -> list[Callable[[float], float]]:
    if type == '3POINT':
        assert legs_cnt == 6
        return [
            traj_shift(T, 0.25*T, 0.5),
            traj_shift(T, 0.75*T, 0.5),
            traj_shift(T, 0.25*T, 0.5),
            traj_shift(T, 0.75*T, 0.5),
            traj_shift(T, 0.25*T, 0.5),
            traj_shift(T, 0.75*T, 0.5),
        ]

    if type == 'RIPPLE':
        assert legs_cnt == 6
        return [
            traj_shift(T, 0.25*T, 3./4.),
            traj_shift(T, 0., 3./4.),
            traj_shift(T, 0.75*T, 3./4.),
            traj_shift(T, 0.75*T, 3./4.),
            traj_shift(T, 0.5*T, 3./4.),
            traj_shift(T, 0.25*T, 3./4.),
        ]

    if type == 'MECHATRONIC':
        return [
            traj_shift(T, _mod_one(0.25 + i/legs_cnt)*T, (legs_cnt-1)/legs_cnt)
            for i in range(legs_cnt)
        ]

    raise Exception(f"Gait \"{type}\" not known")
