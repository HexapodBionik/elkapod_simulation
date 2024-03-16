from typing import Callable


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


def build_gait(type: str, T: float):
    if type == '3POINT':
        return [
            traj_shift(T, 0.25*T, 0.5),
            traj_shift(T, 0.75*T, 0.5),
            traj_shift(T, 0.25*T, 0.5),
            traj_shift(T, 0.75*T, 0.5),
            traj_shift(T, 0.25*T, 0.5),
            traj_shift(T, 0.75*T, 0.5),
        ]

    if type == 'MECHATRONIC':
        return [
            traj_shift(T, (0.25 + 1./6.)*T, 5./6.),
            traj_shift(T, (0.25 + 2./6.)*T, 5./6.),
            traj_shift(T, (0.25 + 3./6.)*T, 5./6.),
            traj_shift(T, (0.25 + 4./6.)*T, 5./6.),
            traj_shift(T, (5./6. - 0.75)*T, 5./6.),
            traj_shift(T, 0.25*T, 5./6.),
        ]

    if type == 'RIPPLE':
        return [
            traj_shift(T, 0.25*T, 3./4.),
            traj_shift(T, 0., 3./4.),
            traj_shift(T, 0.75*T, 3./4.),
            traj_shift(T, 0.75*T, 3./4.),
            traj_shift(T, 0.5*T, 3./4.),
            traj_shift(T, 0.25*T, 3./4.),
        ]

    raise Exception(f"Gait \"{type}\" not known")
