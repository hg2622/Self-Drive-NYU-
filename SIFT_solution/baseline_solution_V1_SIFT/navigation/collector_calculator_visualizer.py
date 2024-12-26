#!/usr/bin/env python3
import rospy
from std_msgs.msg import String
from std_msgs.msg import Float32
import pandas as pd
import os
from math import cos, sin, pi
import matplotlib.pyplot as plt
import numpy as np

class RobotTrajectory:
    def __init__(self, version_index):
        self.version_index = version_index
        self.velocity_data_path = f'/home/linux/catkin_sim_ws/src/navigation/velocity_data_{version_index}.csv'
        self.coordinates = []

    def time_cb(self, data):
        self.time_ros = float(data.data)
        self.time_sys = pd.Timestamp.now()

    def angular_vel_cb(self, data):
        self.ang_vel = data.data

    def linear_vel_cb(self, data):
        self.lin_vel = data.data

    def collect_data(self):
        rospy.init_node('listener')
        rate = 10  # 10 Hz
        rospy.Subscriber("time_publisher", String, self.time_cb)
        rospy.Subscriber("angular_publisher", Float32, self.angular_vel_cb)
        rospy.Subscriber("linear_publisher", Float32, self.linear_vel_cb)

        append_mode = 'w'
        x, y, theta = 0.0, 0.0, 0

        # Setting up live plotting
        plt.ion()
        fig, ax = plt.subplots()
        line, = ax.plot(x, y, label='Robot Trajectory', color='blue')
        start, = ax.plot(x, y, 'go', label='Start')
        end, = ax.plot(x, y, 'rx', label='End')

        ax.set_xlabel('X Coordinate')
        ax.set_ylabel('Y Coordinate')
        ax.set_title('Real-Time Robot Trajectory')
        ax.legend()
        ax.grid(True)
        ax.axis('equal')

        with open(self.velocity_data_path, 'a') as file:
            if append_mode == 'w':
                file.write("timestamp,angular_velocity,linear_velocity\n")
                append_mode = 'a'
            while not rospy.is_shutdown():
                if self.time_sys and self.ang_vel and self.lin_vel:
                    file.write(f"{self.time_sys},{self.ang_vel},{self.lin_vel}\n")
                    # Calculate new position
                    delta_t = 1 / 10  # time step is based on the rate
                    delta_x = self.lin_vel * cos(theta) * delta_t
                    delta_y = self.lin_vel * sin(theta) * delta_t
                    delta_theta = self.ang_vel * delta_t * 0.97
                    x += delta_x
                    y += delta_y
                    theta = (theta + delta_theta) % (2 * pi)
                    
                    # Update the plot
                    line.set_xdata(np.append(line.get_xdata(), x))
                    line.set_ydata(np.append(line.get_ydata(), y))
                    end.set_data(x, y)
                    if len(line.get_xdata()) == 1:  # only at the start
                        start.set_data(x, y)
                    fig.canvas.draw()
                    fig.canvas.flush_events()
                rospy.sleep(1 / rate)

if __name__ == '__main__':
    robot_traj = RobotTrajectory("test_version")
    robot_traj.collect_data()
