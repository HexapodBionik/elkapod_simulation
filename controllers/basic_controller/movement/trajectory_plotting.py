from MotionPlanning.kinematics.kinematics_utils import rot_z, rot_x, rot_y
from movement_leg_trajectory import LegTrajectory
import matplotlib.pyplot as plt
import numpy as np
import matplotlib
matplotlib.use('Qt5Agg')

i = 0
step_length = 0.08
p = np.array([0.2, -step_length / 2 + (1+i) % 2 * step_length, -0.15])
print(p)

print(p)
movement_time = 6
trajectory = LegTrajectory(1, i, step_length, 0.025, movement_time, 1/2, p)
trajectory.generate_symmetrical_trajectory()

trajectory2 = LegTrajectory(0, i, step_length, 0.025, movement_time, 1/2, p)
trajectory2.generate_symmetrical_trajectory()

x = np.arange(0, movement_time, 0.005)


datas = [trajectory.get_position(time_step, 0) for time_step in x]
rotated = [trajectory2.get_position(time_step, np.pi/6) for time_step in x]
x_data = np.array([data[0] for data in datas])
y_data = np.array([data[1] for data in datas])
z_data = np.array([data[2] for data in datas])

x_rotated_data = np.array([data[0] for data in rotated])
y_rotated_data = np.array([data[1] for data in rotated])
z_rotated_data = np.array([data[2] for data in rotated])

fig = plt.figure(figsize=(10, 10))
ax = fig.add_subplot(111, projection='3d')
ax.scatter(x_data, y_data, z_data, marker='o', color="blue", label="Spline polynomial")
ax.scatter(x_rotated_data, y_rotated_data, z_rotated_data, marker='o', color="red", label="Spline polynomial")
plt.show()

