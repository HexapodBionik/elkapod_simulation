from controller import Robot
from time import time
import math

# Create robot instance.
robot = Robot()

# Get the time step of the current world.
timestep = int(robot.getBasicTimeStep())

# Obtaining instances of the devices of the robot.
servo11 = robot.getDevice("Servo11")
servo12 = robot.getDevice("Servo12")
servo13 = robot.getDevice("Servo13")

servo21 = robot.getDevice("Servo21")
servo22 = robot.getDevice("Servo22")
servo23 = robot.getDevice("Servo23")

servo31 = robot.getDevice("Servo31")
servo32 = robot.getDevice("Servo32")
servo33 = robot.getDevice("Servo33")

servo41 = robot.getDevice("Servo41")
servo42 = robot.getDevice("Servo42")
servo43 = robot.getDevice("Servo43")

servo51 = robot.getDevice("Servo51")
servo52 = robot.getDevice("Servo52")
servo53 = robot.getDevice("Servo53")

servo61 = robot.getDevice("Servo61")
servo62 = robot.getDevice("Servo62")
servo63 = robot.getDevice("Servo63")


class Leg:
    def __init__(self, s1, s2, s3, offset, leg_name: str, sign):
        self._s1 = s1
        self._s2 = s2
        self._s3 = s3
        self._offset = offset
        self._leg_name = leg_name
        self._sign = sign

    def __str__(self):
        return self._leg_name

    def complete_move(self):
        self._s2.setPosition(math.pi/5)
        yield 0.1

        self._s1.setPosition(self._offset + (math.pi / 6)*self._sign)
        yield 0.1

        self._s2.setPosition(math.pi / 6 - 0.01)
        yield 0.1

        self._s1.setPosition(self._offset)
        yield 0.1

        self._s2.setPosition(math.pi / 6)
        yield 0.1

    def fine_forward(self):
        self._s2.setPosition(math.pi/5)
        yield 1

        self._s1.setPosition(self._offset + (math.pi/14)*self._sign)
        yield 1

        self._s2.setPosition(math.pi/6)
        yield 1

    def fine_back(self):
        self._s2.setPosition(math.pi*2/15)
        yield 1

        self._s1.setPosition(self._offset - (math.pi/14)*self._sign)
        yield 1

        self._s2.setPosition(math.pi/6)
        yield 1


leg1 = Leg(servo11, servo12, servo13, 0, "l1", -1)
leg2 = Leg(servo21, servo22, servo23, 0, "l2", -1)
leg3 = Leg(servo31, servo32, servo33, 0, "l3", -1)
leg4 = Leg(servo41, servo42, servo43, math.pi, "l4", 1)
leg5 = Leg(servo51, servo52, servo53, math.pi, "l5", 1)
leg6 = Leg(servo61, servo62, servo63, math.pi, "l6", 1)


def main(steps):
    step = steps.pop()
    moves_of_leg = [mol() for mol in step]

    while True:
        t0 = time()
        exec_times = [0 for _ in range(len(step))]
        for i, leg_move in enumerate(step):
            if time() - t0 < exec_times[i]:
                continue

            move_duration = None
            while move_duration is None:
                try:
                    move_duration = next(moves_of_leg[i])
                except StopIteration:
                    move_duration = float('Inf')
            exec_times[i] += move_duration

        waiting_time = min(exec_times)
        if waiting_time == float('Inf'):
            steps.insert(0, step)
            step = steps.pop()
            moves_of_leg = [mol() for mol in step]
            continue

        while time() - t0 < waiting_time:
            robot.step(timestep)
