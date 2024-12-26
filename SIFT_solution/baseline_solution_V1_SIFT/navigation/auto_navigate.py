#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Twist
import sys, select, os
if os.name == 'nt':
  import msvcrt, time
else:
  import tty, termios
from std_msgs.msg import Time
from std_msgs.msg import String
from std_msgs.msg import Float32
import pandas as pd
import json
from datetime import datetime

BURGER_MAX_LIN_VEL = 0.22
BURGER_MAX_ANG_VEL = 2.5

WAFFLE_MAX_LIN_VEL = 0.26
WAFFLE_MAX_ANG_VEL = 1.82

LIN_VEL_STEP_SIZE = 0.22
ANG_VEL_STEP_SIZE = 2.5

msg = """
Start Recreating Trajectory
"""
version_index = "demo_2"


e = """
Communications Failed
"""
def findTargetCounter(vel_file_path, annotation_filepath, goal_image_index):
    json_file_path = annotation_filepath + str(goal_image_index).zfill(8) + ".json"
    # Open and read the JSON file
    with open(json_file_path, 'r') as file:
        # Load JSON content into a dictionary
        data = json.load(file)
    # Extract the timestamp from the nested structure
    extracted_timestamp = data['annotations'][0]['timestamp']
    extracted_timestamp = datetime.strptime(extracted_timestamp, '%Y-%m-%d %H:%M:%S.%f')
    print(extracted_timestamp)
    # Calculate the absolute difference
    df = pd.read_csv(vel_file_path)
    df['timestamp'] = pd.to_datetime(df['timestamp'])
    df['time_difference'] = (df['timestamp'] - extracted_timestamp).abs()

    # Find the index of the row with the smallest time difference
    closest_match_index = df['time_difference'].idxmin()
    closest_match = df.loc[closest_match_index]
    
    # Output the closest match and its index
    #print("Closest Match Index: ", closest_match_index)
    #print("Closest Match Data: ", closest_match)
    #print("Closest Timestamp: ", closest_match['timestamp'])
    
    # Return the index of the closest timestamp match and the row data
    return closest_match_index, closest_match


def findAverageRate(dataframe):
    dataframe['timestamp'] = pd.to_datetime(dataframe['timestamp'])
    dataframe['delta_t'] = dataframe['timestamp'].diff()
    dataframe = dataframe.dropna(subset=['delta_t'])
    average_delta_t = dataframe['delta_t'].mean().total_seconds()
    average_rate = round(1/average_delta_t)
    return average_rate
def vels(target_linear_vel, target_angular_vel):
    return "currently:\tlinear vel %s\t angular vel %s " % (target_linear_vel,target_angular_vel)

def makeSimpleProfile(output, input, slop):
    if input > output:
        output = min( input, output + slop )
    elif input < output:
        output = max( input, output - slop )
    else:
        output = input

    return output

def constrain(input, low, high):
    if input < low:
      input = low
    elif input > high:
      input = high
    else:
      input = input

    return input

def checkLinearLimitVelocity(vel):
    if turtlebot3_model == "burger":
      vel = constrain(vel, -BURGER_MAX_LIN_VEL, BURGER_MAX_LIN_VEL)
    elif turtlebot3_model == "waffle" or turtlebot3_model == "waffle_pi":
      vel = constrain(vel, -WAFFLE_MAX_LIN_VEL, WAFFLE_MAX_LIN_VEL)
    else:
      vel = constrain(vel, -BURGER_MAX_LIN_VEL, BURGER_MAX_LIN_VEL)

    return vel

def checkAngularLimitVelocity(vel):
    if turtlebot3_model == "burger":
      vel = constrain(vel, -BURGER_MAX_ANG_VEL, BURGER_MAX_ANG_VEL)
    elif turtlebot3_model == "waffle" or turtlebot3_model == "waffle_pi":
      vel = constrain(vel, -WAFFLE_MAX_ANG_VEL, WAFFLE_MAX_ANG_VEL)
    else:
      vel = constrain(vel, -BURGER_MAX_ANG_VEL, BURGER_MAX_ANG_VEL)

    return vel

if __name__=="__main__":
    if os.name != 'nt':
        settings = termios.tcgetattr(sys.stdin)

    rospy.init_node('turtlebot3_teleop')
    pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)

    turtlebot3_model = rospy.get_param("model", "burger")
    vel_file_path = f'/home/hanwen/catkin_ws/src/NYU-Self-Drive_F24/baseline_solution_V1_SIFT/navigation/velocity_data_{version_index}.csv'
    annotation_filepath = f'/home/hanwen/catkin_ws/src/NYU-Self-Drive_F24/baseline_solution_V1_SIFT/dataset/annotations/{version_index}/'
    target_linear_vel   = 0.0
    target_angular_vel  = 0.0
    control_linear_vel  = 0.0
    control_angular_vel = 0.0
    rate = rospy.Rate(10) #10hz

    # read velocity file and convert to panda dataframe
    df = pd.read_csv(vel_file_path)
    df['timestamp'] = pd.to_datetime(df['timestamp'])
    timestamps = df['timestamp']
    linear_velocities = df['linear_velocity']
    angular_velocities = df['angular_velocity']

    # get stop position
    goal_id = 326
    stop_counter, stop_timestamp = findTargetCounter(vel_file_path, annotation_filepath, goal_id)

    #print(stop_counter, stop_timestamp)
    #stop_counter = 200
    counter = 1

    # get rate
    rate = 10

    try:
        print(msg)
        while not rospy.is_shutdown():
            # read target velocity
            time_sys = timestamps.iloc[counter]
            v_linear = linear_velocities.iloc[counter]
            v_angular = angular_velocities.iloc[counter]
            
            target_linear_vel = v_linear
            target_angular_vel = v_angular
          
            # publish twist
            twist = Twist()

            control_linear_vel = makeSimpleProfile(control_linear_vel, target_linear_vel, 0.22)
            twist.linear.x = control_linear_vel; twist.linear.y = 0.0; twist.linear.z = 0.0

            control_angular_vel = makeSimpleProfile(control_angular_vel, target_angular_vel, 2.5)
            twist.angular.x = 0.0; twist.angular.y = 0.0; twist.angular.z = control_angular_vel
            rospy.loginfo(f"Sys: {time_sys} | Ang Vel: {v_angular} | Lin Vel: {v_linear}")

            pub.publish(twist)
            rospy.sleep(1/rate)

            if counter < stop_counter:
                counter+=1
            else:
                break
            
    except:
        print(e)

    finally:
        twist = Twist()
        twist.linear.x = 0.0; twist.linear.y = 0.0; twist.linear.z = 0.0
        twist.angular.x = 0.0; twist.angular.y = 0.0; twist.angular.z = 0.0
        pub.publish(twist)

    if os.name != 'nt':
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
