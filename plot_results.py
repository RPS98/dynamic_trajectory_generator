import pandas as pd
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import numpy as np

# Read the CSV file
file_path = "trajectory_new.csv"  # Replace with the path to your file
data = pd.read_csv(file_path)

# Extract columns
time = data["time"].to_numpy()
x, y, z = data["x"].to_numpy(), data["y"].to_numpy(), data["z"].to_numpy()
vx, vy, vz = data["vx"].to_numpy(), data["vy"].to_numpy(), data["vz"].to_numpy()
ax, ay, az = data["ax"].to_numpy(), data["ay"].to_numpy(), data["az"].to_numpy()

# Compute velocity and acceleration magnitudes
velocity_magnitude = np.sqrt(vx**2 + vy**2 + vz**2)
acceleration_magnitude = np.sqrt(ax**2 + ay**2 + az**2)

# Create subplots for position, velocity, acceleration, and their magnitudes
fig, axs = plt.subplots(5, 1, figsize=(10, 18), sharex=True)

# Position
axs[0].plot(time, x, label="x", color="red")
axs[0].plot(time, y, label="y", color="green")
axs[0].plot(time, z, label="z", color="blue")
axs[0].set_title("Position")
axs[0].set_ylabel("Position (m)")
axs[0].legend()
axs[0].grid()

# Velocity
axs[1].plot(time, vx, label="vx", color="red")
axs[1].plot(time, vy, label="vy", color="green")
axs[1].plot(time, vz, label="vz", color="blue")
axs[1].set_title("Velocity")
axs[1].set_ylabel("Velocity (m/s)")
axs[1].legend()
axs[1].grid()

# Acceleration
axs[2].plot(time, ax, label="ax", color="red")
axs[2].plot(time, ay, label="ay", color="green")
axs[2].plot(time, az, label="az", color="blue")
axs[2].set_title("Acceleration")
axs[2].set_ylabel("Acceleration (m/s²)")
axs[2].legend()
axs[2].grid()

# Velocity Magnitude
axs[3].plot(time, velocity_magnitude, label="|v|", color="purple")
axs[3].set_title("Velocity Magnitude")
axs[3].set_ylabel("Velocity (m/s)")
axs[3].legend()
axs[3].grid()

# Acceleration Magnitude
axs[4].plot(time, acceleration_magnitude, label="|a|", color="orange")
axs[4].set_title("Acceleration Magnitude")
axs[4].set_xlabel("Time (s)")
axs[4].set_ylabel("Acceleration (m/s²)")
axs[4].legend()
axs[4].grid()

# Adjust layout
plt.tight_layout()

# Create 3D plot for trajectory
fig_3d = plt.figure(figsize=(8, 8))
ax_3d = fig_3d.add_subplot(111, projection='3d')
ax_3d.plot(x, y, z, label="Trajectory", color="purple")
ax_3d.set_title("3D Trajectory")
ax_3d.set_xlabel("X (m)")
ax_3d.set_ylabel("Y (m)")
ax_3d.set_zlabel("Z (m)")
ax_3d.legend()
ax_3d.grid()

# Show the plots
plt.show(block=False)
plt.pause(0.001)
# Wait for user input to close all figures
try:
    input()
except KeyboardInterrupt:
    pass
plt.close('all')
