from controller import Robot
from MotionPlanning.kinematics.kinematics_solvers import KinematicsSolver
import numpy as np


class Leg:
    """! Leg of MegaLynx robot.

    Provides interface for MegaLynx robot.
    """

    def __init__(self, s1, s2, s3, transform, kinematics_solver):
        """
        @param s1: Servo 1 instance
        @param s2: Servo 2 instance
        @param s3: Servo 3 instance
        @param transform: Coordinate system transform to be applied
        """
        self._s1 = s1
        self._s2 = s2
        self._s3 = s3

        self.transform = transform
        self.kinematics_solver = kinematics_solver

    def set_angles(self, q: np.ndarray) -> None:
        q = np.deg2rad(q)
        q = self.transform(q)

        self._s1.setPosition(q[0])
        self._s2.setPosition(q[1])
        self._s3.setPosition(q[2])

    def set_pose(self, p: np.ndarray) -> None:
        q = self.kinematics_solver.inverse(p)
        self.set_angles(q)


class MegaLynx:
    """
    MegaLynx robot hardware interface
    """

    def __init__(self):
        # Webots simulation robot instance
        self.webots_robot = Robot()

        # Simulation timestep
        self.timestep = int(self.webots_robot.getBasicTimeStep()) // 2

        # Retrieve servos
        servo11 = self.webots_robot.getDevice("Servo11")
        servo12 = self.webots_robot.getDevice("Servo12")
        servo13 = self.webots_robot.getDevice("Servo13")
        servo21 = self.webots_robot.getDevice("Servo21")
        servo22 = self.webots_robot.getDevice("Servo22")
        servo23 = self.webots_robot.getDevice("Servo23")
        servo31 = self.webots_robot.getDevice("Servo31")
        servo32 = self.webots_robot.getDevice("Servo32")
        servo33 = self.webots_robot.getDevice("Servo33")
        servo41 = self.webots_robot.getDevice("Servo41")
        servo42 = self.webots_robot.getDevice("Servo42")
        servo43 = self.webots_robot.getDevice("Servo43")
        servo51 = self.webots_robot.getDevice("Servo51")
        servo52 = self.webots_robot.getDevice("Servo52")
        servo53 = self.webots_robot.getDevice("Servo53")
        servo61 = self.webots_robot.getDevice("Servo61")
        servo62 = self.webots_robot.getDevice("Servo62")
        servo63 = self.webots_robot.getDevice("Servo63")

        # Hexapod dimensions
        mount_t = np.array([0, 0, 0.05])
        t1 = np.array([0.063, 0, 0])
        t2 = np.array([0.09, 0, 0])
        t3 = np.array([0.23, 0, 0])

        solver = KinematicsSolver(mount_t, t1, t2, t3)

        # Create leg objects
        leg1 = Leg(servo11, servo12, servo13,
                   lambda q: q*np.array([-1., 1., 1.]),
                   solver)
        leg2 = Leg(servo21, servo22, servo23,
                   lambda q: q*np.array([-1., 1., 1.]),
                   solver)
        leg3 = Leg(servo31, servo32, servo33,
                   lambda q: q*np.array([-1., 1., 1.]),
                   solver)
        leg4 = Leg(servo41, servo42, servo43,
                   lambda q: q+np.array([np.pi, 0., 0.]),
                   solver)
        leg5 = Leg(servo51, servo52, servo53,
                   lambda q: q+np.array([np.pi, 0., 0.]),
                   solver)
        leg6 = Leg(servo61, servo62, servo63,
                   lambda q: q+np.array([np.pi, 0., 0.]),
                   solver)

        # Store legs in self.legs list
        self.legs = [leg1, leg2, leg3, leg4, leg5, leg6]

    def simulation_step(self):
        return self.webots_robot.step(self.timestep)
