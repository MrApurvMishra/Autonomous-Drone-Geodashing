## AER1217 Winter 2022, Project
### Team E: Aditya Jain, Apurv Mishra, Praharsha Abbireddy


**Runnning the code **

Run 'roslaunch aer1217_ardrone_simulator ardrone_simulator.launch' in terminal. This will launch all the required nodes.



**Files Information **

1. position_controller.py - calculates the commanded control using the vicon feedback, quadrotor dynamics and desired trajectory
                        
2. ros_interface.py - subscribes to vicon feedback and desired trajectory topic, calls the position_controller and publishes the required control to /cmd_vel_RHC

3. indoor_robotics_lab_interface.py - subscribes to /cmd_vel_RHC, calculates required motor speeds and publishes it

4. desired_positions.py - generates the trajectory waypoints and publishes it to /trajectory_generator

5. ardrone_simulator.launch - launches all the required nodes for the simulation to occur

6. bag_to_data.py - reads relevant topics from the bag file, and saves the image and corresponding pose data to a JSON file

7. detect_obstacles.py - uses Hough transform to detect circles and saves the data to the JSON file, corresponding to each frame

8. detect_landmarks.py - Using SIFT and brute-force method to match and detect targets and saves the data to the JSON file, corresponding to each frame

9. georeferencing.py - reads the JSON file with all the image and pose data, uses this data and provided camera matrices to find the circle centres and landmark positions in the inertial frame 

10. obstacle_detection_info.json - detection information for the circles and synchronized drone pose

11. landmark_detection_info.json - detection information for the landmarks and synchronized drone pose

12. targetdata_to_json.py - saves the obstacle and landmark information to a json file

12. target_data.json - file containing the final georeferenced pose and scale information of the obstacles and landmarks

13. occupancy.py - script to build the occupancy map using the obstacle data

14. path_planner.py - Dijkstra's algorithm implementation
