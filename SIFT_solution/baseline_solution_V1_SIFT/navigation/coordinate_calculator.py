#!/usr/bin/env python3
import pandas as pd
import numpy as np
from math import cos, sin, pi
import os


version_index = "demo_1"
current_directory = os.getcwd()
vel_file_path = f'{current_directory}/src/NYU-Self-Drive_F24/baseline_solution_V1_SIFT/velocity_data_{version_index}.csv'

# Read CSV file containing time series data
df = pd.read_csv(vel_file_path)

# Convert 'timestamp' column to datetime
df['timestamp'] = pd.to_datetime(df['timestamp'])

# Extract timestamps, linear velocities, and angular velocities
timestamps = df['timestamp']
linear_velocities = df['linear_velocity']
angular_velocities = df['angular_velocity']

# Initialize position and orientation
x = 0.0
y = 0.0
theta = 0

# Initialize list to store coordinates
coordinates = []

# Initialize previous timestamp
prev_timestamp = timestamps.iloc[0]

# Iterate over time steps
for i in range(1, len(timestamps)):
    # Calculate time step size
    delta_t = (timestamps.iloc[i] - prev_timestamp).total_seconds()
    
    # Get linear and angular velocities at current time step
    v_linear = linear_velocities.iloc[i]
    v_angular = angular_velocities.iloc[i]
    
    # Calculate change in position and orientation
    delta_x = v_linear * cos(theta) * delta_t
    delta_y = v_linear * sin(theta) * delta_t
    delta_theta = v_angular * delta_t * 1.1
    
    # Update position and orientation
    x += delta_x
    y += delta_y
    theta += delta_theta
    
    # Ensure theta is within [0, 2*pi) range
    theta = theta % (2 * pi)
    
    # Append coordinates to list
    coordinates.append({'timestamp': timestamps.iloc[i], 'x': x, 'y': y, 'theta': theta})
    
    # Update previous timestamp
    prev_timestamp = timestamps.iloc[i]

# Convert list of dictionaries to DataFrame
coordinates_df = pd.DataFrame(coordinates)

# Write DataFrame to CSV file
current_directory = os.getcwd()
file_path = f'{current_directory}/src/NYU-Self-Drive_F24/baseline_solution_V1_SIFT/robot_coordinates_{version_index}.csv'
if os.path.exists(file_path):
    os.remove(file_path)
coordinates_df.to_csv(file_path, index=False)