from time import time
import numpy as np
import math
from controller import Robot
from ..simulation.leg import Leg

robot = Robot()


timestep = int(robot.getBasicTimeStep())


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