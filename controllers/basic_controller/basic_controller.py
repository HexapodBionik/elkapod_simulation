"""basic_controller controller."""

# You may need to import some classes of the controller module. Ex:
#  from controller import Robot, Motor, DistanceSensor
from controller import Robot
from time import time
#from leg_movement import LegMovementTrajectory2
import numpy as np
from MotionPlanning.kinematics.kinematics_solvers import KinematicsSolver
from MotionPlanning.kinematics.kinematics_utils import rot_z
from simulation.leg import Leg
from movement.movement_leg_trajectory import LegTrajectory
import math
robot = Robot()

# get the time step of the current world.
timestep = int(robot.getBasicTimeStep())

# You should insert a getDevice-like function in order to get the
# instance of a device of the robot. Something like:
#  motor = robot.getDevice('motorname')
#  ds = robot.getDevice('dsname')
#  ds.enable(timestep)
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


leg1 = Leg(servo11, servo12, servo13, 0, "l1", -1)
leg2 = Leg(servo21, servo22, servo23, 0, "l2", -1)
leg3 = Leg(servo31, servo32, servo33, 0, "l3", -1)
leg4 = Leg(servo41, servo42, servo43, math.pi, "l4", 1)
leg5 = Leg(servo51, servo52, servo53, math.pi, "l5", 1)
leg6 = Leg(servo61, servo62, servo63, math.pi, "l6", 1)

leg_list = [leg1, leg2, leg3, leg4, leg5, leg6]
#leg_list.reverse()


leg_trajectories = []
movement_time = 6
for i in range(6):
    p = np.array([0.2, 0.04 - i / 5 * 0.08, -0.1])
    trajectory = LegTrajectory(i, 0.08, 0.025, movement_time, 1/6, p)
    trajectory.generate_symmetrical_trajectory()
    leg_trajectories.append(trajectory)


mount_t = np.array([0, 0, 0.05])
t1 = np.array([0.063, 0, 0])
t2 = np.array([0.09, 0, 0])
t3 = np.array([0.23, 0, 0])

kinematics_solver = KinematicsSolver(mount_t, t1, t2, t3)

i = 0

t_start = time()
while robot.step(timestep) != -1 and time() - t_start < 3:
    for traj, leg in zip(leg_trajectories, leg_list):
        p = traj.get_position(0)
        #p = p @ rot_z(-np.pi/2)
        q = kinematics_solver.inverse(p)
        # print(q)
        leg.set_angles(q)

time1 = time()
while robot.step(timestep) != -1:
    t2 = time()
    if t2-time1 < movement_time:
        for traj, leg in zip(leg_trajectories, leg_list):
            p = traj.get_position(t2 - time1)
            #p = p @ rot_z(-np.pi / 2)
            q = kinematics_solver.inverse(p)
            #print(q)
            leg.set_angles(q)
    else:
        time1 = t2

    """# Regulacja wysokości kadłuba
    if i < 200:
        p = np.array([0.2, 0, -0.22 + float(i / 1000)])
        q = kinematics_solver.inverse(p)
        # print(q)
        for leg in leg_list:
            leg.set_angles(np.deg2rad(q[0]), np.deg2rad(q[1]), np.deg2rad(q[2]))
        i += 1"""


