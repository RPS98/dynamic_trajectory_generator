import trajectory_generator as dtb
import numpy as np

# Instance the dynamic trajectory class
trajectory_bind = dtb.DynamicTrajectory()

# Initial position of the vehicle
initial_position = np.array([0.0, 0.0, 1.0])
initial_yaw = 0.0

# Trajectory waypoints
waypoints = [
    np.array([1.0, 1.0, 1.0]),
    np.array([1.0, -1.0, 1.0]),
    np.array([-1.0, -1.0, 1.0]),
    np.array([-1.0, 1.0, 1.0]),
    np.array([0.0, 0.0, 1.0])
]

# Generate trajectory
max_velocity = 2.0
trajectory_bind.set_path_facing(True)
trajectory_bind.generate_trajectory(
    initial_position, initial_yaw,
    waypoints, max_velocity)

# Get max and min time
max_time = trajectory_bind.get_max_time()
min_time = trajectory_bind.get_min_time()

# Evaluate trajectory
time = min_time
position = np.zeros(3)
velocity = np.zeros(3)
acceleration = np.zeros(3)
yaw = 0.0

dt = 0.01
while time < max_time:
    position, velocity, acceleration, yaw = \
        trajectory_bind.evaluate_trajectory(time)
    time += dt

print("Trajectory generated from: ", initial_position)
print("And yaw: ", initial_yaw)
print("Generate trajectory with max velocity: ", max_velocity)
print("Generate trajectory for waypoints: ")
for i in range(len(waypoints)):
    print(waypoints[i])

print("Trajectory generated with:")
print("Min time: ", min_time)
print("Max time: ", max_time)

print("Trajectory final references:")
print("Position: ", position)
print("Velocity: ", velocity)
print("Acceleration: ", acceleration)
print("Yaw: ", yaw)

