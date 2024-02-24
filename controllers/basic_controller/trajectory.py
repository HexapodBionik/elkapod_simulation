from movement.movement_leg_trajectory import LegTrajectory
import matplotlib.pyplot as plt
import numpy as np
import matplotlib
matplotlib.use('Qt5Agg')

i = 0
step_length = 0.08
p = np.array([0.2, 0.04-i/5*step_length, -0.12])
movement_time = 6
trajectory = LegTrajectory(i, step_length, 0.025, movement_time, 1/6, p)
trajectory.generate_trajectory_2()
x = np.arange(0, movement_time, 0.005)


datas = [trajectory.get_position(time_step) for time_step in x]
x_data = np.array([data[0] for data in datas])
y_data = np.array([data[1] for data in datas])
z_data = np.array([data[2] for data in datas])

fig = plt.figure(figsize=(10, 10))
ax = fig.add_subplot(111, projection='3d')
scatter = ax.scatter(x_data, y_data, z_data, marker='o', label="Spline polynomial")
plt.show()

