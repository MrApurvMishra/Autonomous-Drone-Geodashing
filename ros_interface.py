#!/usr/bin/env python2

"""
ROS Node for controlling the ARDrone 2.0 using the ardrone_autonomy package.

This ROS node subscribes to the following topics:
/vicon/ARDroneCarre/ARDroneCarre

This ROS node publishes to the following topics:
/cmd_vel_RHC

"""

from __future__ import division, print_function, absolute_import
# from asyncio import TimerHandle
from math import cos, asin
from this import d

# Import ROS libraries
import roslib
import rospy
import numpy as np

# Import class that computes the desired positions
from geometry_msgs.msg import TransformStamped, Twist
from position_controller import PositionController


class ROSControllerNode(object):
    """ROS interface for controlling the Parrot ARDrone in the Vicon Lab."""

    # write code here to define node publishers and subscribers
    # publish to /cmd_vel topic
    # subscribe to /vicon/ARDroneCarre/ARDroneCarre for position and attitude feedback
    def __init__(self):
        """Initialize the ROSControllerNode class"""
        # Subscribers
        self.sub_vicon_data = rospy.Subscriber('/vicon/ARDroneCarre/ARDroneCarre',
                                               TransformStamped,
                                               self.calculate_control)

        self.sub_pos_generator = rospy.Subscriber('/trajectory_generator',
                                                  Twist,
                                                  self.desired_positions)

        # Publisher
        self.pub_cmd_vel = rospy.Publisher('/cmd_vel_RHC',
                                           Twist,
                                           queue_size=500)

        # Keeping track of time for numerical differentiation
        self.old_time = rospy.get_time()

        # Instantiating the position controller class
        self.position_controller = PositionController()

        # Publish the control command at the below frequency
        self.control_loop_frequency = 100
        rospy.Timer(rospy.Duration(1 / self.control_loop_frequency), self.publish_control)

        # Initialize messages for publishing
        self.cmd_vel_msg = Twist()

        # Desired positions
        self.des_pos = [0.0, 0.0, 0.0, 0.0]

    def desired_positions(self, traj_data):
        """subscribes to the desired positions in the trajectory"""

        self.des_pos[0] = traj_data.linear.x
        self.des_pos[1] = traj_data.linear.y
        self.des_pos[2] = traj_data.linear.z
        self.des_pos[3] = traj_data.angular.z

    def calculate_control(self, vicon_data):
        """calculates the control based on quadrotor dynamics and vicon feedback"""

        # Determine the time step for differentiation and integration
        current_time = rospy.get_time()
        dt = current_time - self.old_time

        [roll_cmd, pitch_cmd, z_dot_cmd, yaw_dot_cmd] = self.position_controller.dynamics_ctrl_cal(vicon_data,
                                                                                                   self.des_pos, dt)

        self.old_time = current_time

        # setting the values to be published
        self.cmd_vel_msg.linear.x = roll_cmd
        self.cmd_vel_msg.linear.y = pitch_cmd
        self.cmd_vel_msg.linear.z = z_dot_cmd
        self.cmd_vel_msg.angular.z = yaw_dot_cmd

    def publish_control(self, event=None):
        """publishes the control command calculated above"""

        self.pub_cmd_vel.publish(self.cmd_vel_msg)

    # [help function] limit the roll and pitch angle to [-17.5 17.5] degree
    def limit_rp(self, angle_cmd):
        if angle_cmd > 0.3:
            angle_cmd = 0.3
        elif angle_cmd < -0.3:
            angle_cmd = -0.3
        return angle_cmd

    # [help function] wrap the angle error to [-pi, pi]
    def wrap_angle(self, angle_error):
        while (angle_error > np.pi):
            angle_error -= 2 * np.pi
        while (angle_error < -np.pi):
            angle_error += 2 * np.pi
        return angle_error


if __name__ == '__main__':
    # write code to create ROSControllerNode
    rospy.init_node('position_controller')
    ardrone = ROSControllerNode()
    rospy.spin()
