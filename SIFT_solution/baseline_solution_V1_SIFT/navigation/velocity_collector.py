#!/usr/bin/env python3
import rospy
from std_msgs.msg import String
from std_msgs.msg import Float32
import pandas as pd
import os

time_ros = None
time_sys = None
ang_vel = None
lin_vel = None

def time_cb(data):
    #rospy.loginfo("Time: %s", data.data)
    global time_ros
    time_ros = float(data.data)
    global time_sys
    time_sys = pd.Timestamp.now()

def angular_vel_cb(data):
    #rospy.loginfo("Angular: %s", data.data)
    global ang_vel
    ang_vel = data.data

def linear_vel_cb(data):
    #rospy.loginfo("Linear: %s", data.data)
    global lin_vel
    lin_vel = data.data
    #rospy.loginfo(f"ROS: {time_ros} | Sys: {time_sys} | Ang Vel: {ang_vel} | Lin Vel: {lin_vel}")

def listener():

    # In ROS, nodes are uniquely named. If two nodes with the same
    # name are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    version_index = "demo_2"
    rospy.init_node('listener')
    rate = 10 #hz

    append_mode = 'w'

    # initialize file before starting with the version index, which could be a date as well
    current_directory = os.getcwd()
    file_path = f'{current_directory}/velocity_data_{version_index}.csv'
    if os.path.exists(file_path):
        os.remove(file_path)

    while not rospy.is_shutdown():
        # update data using subscribers
        rospy.Subscriber("time_publisher", String, time_cb)
        rospy.Subscriber("angular_publisher", Float32, angular_vel_cb)
        rospy.Subscriber("linear_publisher", Float32, linear_vel_cb)

        # write to file with a header block
        with open(file_path, 'a') as file:
            if append_mode == 'w':
                # info on the first line can be customized
                # first row of data omitted for consistency
                file.write("timestamp,angular_velocity,linear_velocity\n")
                append_mode = 'a'
            else:
                file.write(f"{time_sys},{ang_vel},{lin_vel}\n")
        print(f"ROS: {time_ros} | Sys: {time_sys} | Ang Vel: {ang_vel} | Lin Vel: {lin_vel}")

        rospy.sleep(1/rate)

    # spin() simply keeps python from exiting until this node is stopped
    #rospy.spin()
    

if __name__ == '__main__':
    listener()
    
    