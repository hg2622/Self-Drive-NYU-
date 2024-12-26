#!/usr/bin/env python3
import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
from math import cos, sin, pi
import os

version_index = "demo_1"
current_directory = os.getcwd()
file_path = f'{current_directory}/src/NYU-Self-Drive_F24/baseline_solution_V1_SIFT/robot_coordinates_{version_index}.csv'

# Read CSV file containing robot coordinates
coordinates_df = pd.read_csv(file_path)

# Extract x, y, and theta values
x_values = coordinates_df['x'].values
y_values = coordinates_df['y'].values
theta_values = coordinates_df['theta'].values

# Plot robot's trajectory
plt.figure(figsize=(8, 6))
plt.plot(x_values, y_values, label='Robot Trajectory', color='blue')
plt.scatter(x_values[0], y_values[0], marker='o', color='green', label='Start')
plt.scatter(x_values[-1], y_values[-1], marker='x', color='red', label='End')
plt.xlabel('X Coordinate')
plt.ylabel('Y Coordinate')
plt.title('Robot Trajectory')
plt.legend()
plt.grid(True)
plt.axis('equal')  # Ensure aspect ratio is equal
plt.show()