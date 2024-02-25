from time import time
import numpy as np
import math
from controller import Robot
from basic_controller.simulation.leg_steering import LegSteering
import math
from basic_controller.connection.connection_mockup import HardwareControllerConnection, robot, timestep
import threading


from basic_controller.movement.movement_core import MovementCore
from basic_controller.general.setup_loading import load_setup


setup_data = load_setup("setup/setup.yaml")
connection = HardwareControllerConnection()

core = MovementCore(connection)
core.init(setup_data.get("movement_core"))

movement_time = 50
user_command = False
velocity = 0.05


def simulation():
    global user_command, velocity
    core.generate_trajectories(0.05)
    t1 = time()
    while robot.step(timestep) != -1:
        t2 = time()
        if not core.is_action_finished(t2 - t1):
            core.execute_trajectories(t2 - t1)
        else:
            t1 = time()
            if user_command:
                core.generate_trajectories(velocity)
                user_command = False


def user_input():
    global user_command, velocity
    while True:
        velocity = float(input("Set velocity: "))
        user_command = True


if __name__ == "__main__":
    t1 = threading.Thread(target=simulation)
    t2 = threading.Thread(target=user_input)
    t1.start()
    t2.start()
    t1.join()
    t2.join()

