## How to use the SIFT auto-navigation baseline.

Make sure to change the file path for all scripts. 

Perform actions in the following order:
1. Run **roscore** on remote PC. Run **robot.launch** and **camera.launch** on the turtlebot. 
2. Run **turtlebot3_teleop_key** on remote PC (directly run the python file in the same folder). 
3. Run **camera_store.py** to collect image.
4. Run script **velocity_collector.py**; modify version index and filepath
5. When collection is finished. Close in the following order:
    - 1st camera_store
    - 2nd velocity_collector
    - 3rd teleop
6. Run **sift_v3_tim.py** on the collected data to find the _goal_id_.
7. Input _goal_id_ to **auto_navigate.py**; run script to navigate to destination.
8. Run **coordinate_calculator.py** then **visualize_map.py** to draw map (optional). 