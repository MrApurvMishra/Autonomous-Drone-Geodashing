#!/usr/bin/env python2

"""ROS Node for publishing desired positions."""

from __future__ import division, print_function, absolute_import

# Import libraries
import roslib
import rospy
import numpy as np
import pickle
import os
import json
from geometry_msgs.msg import TransformStamped, Twist, Vector3

# Import class that computes the desired positions
# from aer1217_ardrone_simulator import PositionGenerator


class ROSDesiredPositionGenerator(object):
    """ROS interface for publishing desired positions."""

    def __init__(self):
        # Publisher
        self.pub_traj = rospy.Publisher('/trajectory_generator', Twist, queue_size=500)

        # Subscriber
        self.sub_vicon_data = rospy.Subscriber('/vicon/ARDroneCarre/ARDroneCarre',
                                               TransformStamped,
                                               self.current_position)                                     

        # Drone's current position
        self.x_cur = 0.0
        self.y_cur = 0.0
        self.z_cur = 0.0

        # defining index for array of trajectory points
        self.idx = 0

        # error threshold for waypoint completion
        self.error_range = 0.1

        # defining message type for the publisher
        self.pub_traj_msg = Twist()

        # trajectory data
        self.file_dir        = os.path.dirname(__file__)
        # self.path_data       = pickle.load(open(self.file_dir + '/final_path.pickle','rb'))       
        self.z_des           = 2.0
        self.yaw_des_default = 0.0

        # target data
        self.target_seq           = [0, 3, 2, 1, 4, 0]
        # self.target_seq           = [0, 2, 0]
        self.targetdata_filepath  = self.file_dir + '/target_data.json'
        with open(self.targetdata_filepath, 'r') as f:
            self.targetdata = json.load(f)
        self.target_pos      = self.traj_calculation()

        # Publish the control command at the below frequency
        self.desired_pos_frequency = 10
        rospy.Timer(rospy.Duration(1 / self.desired_pos_frequency), self.publish_trajectory)

    def current_position(self, vicon_data):
        self.x_cur = vicon_data.transform.translation.x
        self.y_cur = vicon_data.transform.translation.y
        self.z_cur = vicon_data.transform.translation.z

    def publish_trajectory(self, event=None):
        x_diff = self.x_cur - self.target_pos[0, self.idx]
        y_diff = self.y_cur - self.target_pos[1, self.idx]
        z_diff = self.z_cur - self.target_pos[2, self.idx]

        error = np.sqrt(x_diff ** 2 + y_diff ** 2 + z_diff ** 2)
        if error < self.error_range:
            self.idx += 1

        self.pub_traj_msg.linear.x = self.target_pos[0, self.idx]
        self.pub_traj_msg.linear.y = self.target_pos[1, self.idx]
        self.pub_traj_msg.linear.z = self.target_pos[2, self.idx]
        self.pub_traj_msg.angular.z = self.target_pos[3, self.idx]

        self.pub_traj.publish(self.pub_traj_msg)

    def traj_calculation(self):
        points_x = []
        points_y = []
        points_z = []
        angles_z = []
        lmark_des_yaw = self.yaw_des_default

        # traversing over all path segments
        for i in range(len(self.target_seq)-1):
            start_idx       = self.target_seq[i]
            goal_idx        = self.target_seq[i+1]
            path_file_name  = '/path_' + str(start_idx) + str(goal_idx) + '.pickle'
            path_data       = pickle.load(open(self.file_dir + path_file_name,'rb')) 

            # traversing over each point in the path
            for j in range(len(path_data)):
                if goal_idx!=0 and j==len(path_data)-1:           
                    lmark_num     = goal_idx
                    lmark_data    = self.targetdata['landmark_' + str(lmark_num)]['pose']
                    lmark_yaw     = lmark_data[5]
                    lmark_des_yaw = lmark_yaw + 1.57   # adding 90 degree to landmark orientation
                # else:
                #     lmark_des_yaw = self.yaw_des_default
                
                points_x.append(path_data[j][0]) 
                points_y.append(path_data[j][1])
                points_z.append(self.z_des)
                angles_z.append(lmark_des_yaw)           

        # update trajectory and return
        self.target_pos = np.asarray([points_x, points_y, points_z, angles_z])

        return self.target_pos


if __name__ == '__main__':
    rospy.init_node('desired_position')
    ROSDesiredPositionGenerator()
    rospy.spin()