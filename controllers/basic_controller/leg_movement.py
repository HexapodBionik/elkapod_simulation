import numpy as np


def min_max_norm(val, min_val, max_val, new_min, new_max):
  return (val - min_val) * (new_max - new_min) / (max_val - min_val) + new_min



class LegMovementTrajectory2:
    def __init__(self, input_position, leg_movement_nb):
        self._trajectory_phase = 0
        self._input_position = input_position
        self._leg_movement_nb = leg_movement_nb

        self._phase1_func = None
        self._phase2_func = None

        self._swing_y_func = None
        self._swing_z_func = None
        self._movement_unit = None

    def generate(self, step_length: float, step_height: float, movement_time: float):
        movement_unit = movement_time / 6
        self._movement_unit = movement_unit

        phase_1_time = self._leg_movement_nb * movement_unit
        phase_2_time = movement_time - phase_1_time - movement_unit

        if self._leg_movement_nb < 5:
            phase_1_length = (phase_1_time / movement_time) * step_length
        else:
            phase_1_length = step_length

        phase_2_length = (movement_time-phase_1_time)/movement_time*step_length


        # Solve for second part

        try:
            A2 = np.array([
                [phase_1_time + movement_unit, 1],
                [movement_time, 1]
            ])

            b2 = np.array([phase_2_length, 0])
            x2 = np.linalg.solve(A2, b2)
            self._phase2_func = np.poly1d(x2)
        except Exception:
            pass

        if phase_1_time > 0:
            self._phase1_func = np.poly1d(np.array([-phase_1_length / phase_1_time, 0]))

        self._swing_y_func = lambda x: -(step_length/2)+phase_1_length + (step_length/2) * np.cos(x)
        self._swing_z_func = lambda x: step_height * np.sin(x)

    def get_position(self, t: float) -> np.ndarray:
        t = float(t)

        if t < self._leg_movement_nb*self._movement_unit:
            return np.array([self._input_position[0], self._input_position[1]-self._phase1_func(t), self._input_position[2]])

        elif self._leg_movement_nb*self._movement_unit <= t < (self._leg_movement_nb+1)*self._movement_unit:
            t = min_max_norm(t, self._leg_movement_nb*self._movement_unit, (self._leg_movement_nb+1)*self._movement_unit, 0, np.pi)
            y_swing = self._swing_y_func(t)
            z_swing = self._swing_y_func(t)
            return np.array([self._input_position[0], self._input_position[1] + self._swing_y_func(t), self._input_position[2]+self._swing_z_func(t)])
        else:
            return np.array([self._input_position[0], self._input_position[1] - self._phase2_func(t), self._input_position[2]])





class LegMovementTrajectory3:
    def __init__(self, input_position, leg_movement_nb):
        self._trajectory_phase = 0
        self._input_position = input_position
        self._leg_movement_nb = leg_movement_nb

        self._phase1_func = None
        self._phase2_func = None

        self._swing_y_func = None
        self._swing_z_func = None
        self._movement_unit = None

        self._normalized = True

    def generate(self, step_length: float, step_height: float, movement_time: float):
        movement_unit = movement_time / 6
        self._movement_unit = movement_unit

        phase_1_time = self._leg_movement_nb * movement_unit
        phase_2_time = movement_time - phase_1_time - movement_unit

        if self._leg_movement_nb < 5:
            phase_1_length = (phase_1_time / movement_time) * step_length
        else:
            phase_1_length = step_length

        phase_2_length = (movement_time - phase_1_time) / movement_time * step_length

        # Solve for second part

        try:
            A2 = np.array([
                [phase_1_time + movement_unit, 1],
                [movement_time, 1]
            ])

            b2 = np.array([phase_2_length, 0])
            x2 = np.linalg.solve(A2, b2)
            self._phase2_func = np.poly1d(x2)
        except Exception:
            pass

        if phase_1_time > 0:
            self._phase1_func = np.poly1d(np.array([-phase_1_length / phase_1_time, 0]))

        self._swing_y_func = lambda x: (step_length / 2) * np.cos(x)
        self._swing_z_func = lambda x: step_height * np.sin(x)

    def generate_normalized(self, step_length: float, step_height: float, movement_time: float):
        step_length /= 2
        movement_unit = movement_time / 6
        self._movement_unit = movement_unit

        phase_1_time = self._leg_movement_nb * movement_unit
        phase_2_time = movement_time - phase_1_time - movement_unit

        if self._leg_movement_nb < 5:
            phase_1_length = (phase_1_time / movement_time) * step_length
        else:
            phase_1_length = step_length

        if phase_1_time > 0:
            self._phase1_func = np.poly1d(np.array([-phase_1_length / phase_1_time, 0]))

        f = lambda x: 0.1*x + 0.5
        self._swing_y_func = lambda x: -(step_length/2-phase_1_length/2)+(step_length*f(self._leg_movement_nb)) * np.cos(x)
        self._swing_z_func = lambda x: step_height * np.sin(x)

    def get_position(self, t: float) -> tuple[np.ndarray, bool]:
        t = float(t)

        if t < self._leg_movement_nb * self._movement_unit:
            q = np.array(
                [self._input_position[0], self._input_position[1] - self._phase1_func(t),
                 self._input_position[2]])

        elif self._leg_movement_nb * self._movement_unit <= t < (self._leg_movement_nb + 1) * self._movement_unit:
            t_mapped = min_max_norm(t, self._leg_movement_nb * self._movement_unit,
                             (self._leg_movement_nb + 1) * self._movement_unit, 0, np.pi)
            y_swing = self._swing_y_func(t)
            z_swing = self._swing_y_func(t)
            q = np.array([self._input_position[0], self._input_position[1] + self._swing_y_func(t_mapped),
                             self._input_position[2] + self._swing_z_func(t_mapped)])
        else:
            q = np.array(
                [self._input_position[0], self._input_position[1] - self._phase2_func(t), self._input_position[2]])


        generation = False
        if self._normalized and t >= (self._leg_movement_nb + 1) * self._movement_unit:
            generation = True
        elif not self._normalized and t >= self._movement_unit * 6:
            generation = True

        return q, generation








class Movement:
    def __init__(self, kinematics_solver):
        self._step_length = 0.1
        self._swing_part = 0.25
        self._stand_part = 0.75

        assert (self._swing_part + self._stand_part) == 1

    def generate_walk_type(self, velocity):
        t = self._step_length / velocity

        # Determine walk type (to be implemented)

    def wave_gait(self, t, legs: list):
        one_leg = t/6
        one_leg_swing = one_leg*self._swing_part

        """
        Move order:
        L3, L2, L1, R3, R2, R1
        
        R1 - Leg 1
        R2 - Leg 2
        R3 - Leg 3
        L1 - Leg 4
        L2 - Leg 5
        L3 - Leg 6
        """
        order = [5, 4, 3, 2, 1, 0]

        for i in range(6):
            pass



