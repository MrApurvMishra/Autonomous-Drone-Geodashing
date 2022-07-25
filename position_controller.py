#!/usr/bin/env python2

"""Class for writing position controller."""

from __future__ import division, print_function, absolute_import

# Import ROS libraries
import roslib
import rospy
import numpy as np

# Import class that computes the desired positions
from tf.transformations import euler_from_quaternion
from geometry_msgs.msg import TransformStamped, Twist


class PositionController(object):
    """ROS interface for controlling the Parrot ARDrone in the Vicon Lab."""

    # write code here for position controller
    def __init__(self):
        """Initialize the PositionController class"""

        # variables for numerical differentiation
        self.x_old = 0.0;
        self.x_err_old = 0.0
        self.y_old = 0.0;
        self.y_err_old = 0.0
        self.z_old = 0.0;
        self.z_err_old = 0.0
        self.yaw_old = 0.0;
        self.yaw_err_old = 0.0
        self.z_dot_old = 0.0

        # PD parameters
        self.kp_xy = 4  # Kp for x and y controller
        self.kd_xy = 8.0  # Kd for x and y controller
        self.kp_z = 6.0  # Kp for z controller
        self.kp_yaw = 4.0  # Kp for yaw controller

        # gravity
        self.g = 9.8

        # test reference point; should come from the subscribed topic
        self.yaw_des = 0.0

        # calculates control output using quadrotor dynamics, desired positions and vicon feedback

    def dynamics_ctrl_cal(self, data, des_pos, dt):

        # Convert to quaternion object for use by euler_from_quaternion()
        quaternion = np.array([data.transform.rotation.x,
                               data.transform.rotation.y,
                               data.transform.rotation.z,
                               data.transform.rotation.w])

        # Determine the euler angles
        euler = euler_from_quaternion(quaternion)
        roll_cur = euler[0]
        pitch_cur = euler[1]
        yaw_cur = euler[2]

        # Determine the current translation position
        x_cur = data.transform.translation.x
        y_cur = data.transform.translation.y
        z_cur = data.transform.translation.z

        # Desired transalation position
        x_des = des_pos[0]
        y_des = des_pos[1]
        z_des = des_pos[2]

        # desired yaw
        self.yaw_des = des_pos[3]

        # calculate error
        x_err = x_des - x_cur
        y_err = y_des - y_cur
        z_err = z_des - z_cur
        yaw_err = self.yaw_des - yaw_cur

        # normalize yaw error to [-pi, pi]
        yaw_err = self.wrap_angle(yaw_err)

        # ------------------------------ PD Controller ------------------------------ #
        # First order system for z and yaw
        z_dot_cmd = self.kp_z * z_err
        yaw_dot_cmd = self.kp_yaw * yaw_err

        # Second order system for x and y (PD control)
        x_dot_dot_cmd = self.kp_xy * x_err + self.kd_xy * (x_err - self.x_err_old) / dt
        y_dot_dot_cmd = self.kp_xy * y_err + self.kd_xy * (y_err - self.y_err_old) / dt

        # calculate roll and pitch command from x,y acceleration commands
        # use measured z_dot_dot (numerical differentiation from vicon) to compute f
        z_dot = (z_cur - self.z_old) / dt
        z_dot_dot = (z_dot - self.z_dot_old) / dt
        f = (z_dot_dot + self.g) / (np.cos(roll_cur) * np.cos(pitch_cur))

        # compute roll_cmd
        roll_cmd_sin = - y_dot_dot_cmd / f
        # limit the roll cmd to [-17.5 17.5] degree
        roll_cmd_sin = self.limit_rp(roll_cmd_sin)
        roll_cmd = np.arcsin(roll_cmd_sin)

        # compute pitch_cmd
        pitch_cmd_sin = x_dot_dot_cmd / (f * np.cos(roll_cmd))
        # limit the pitch cmd to [-17.5 17.5] degree
        pitch_cmd_sin = self.limit_rp(pitch_cmd_sin)
        pitch_cmd = np.arcsin(pitch_cmd_sin)

        # convert roll/pitch from inertial world frame into drone body frame
        roll_cmd_body = roll_cmd * np.cos(yaw_cur) + pitch_cmd * np.sin(yaw_cur)
        pitch_cmd_body = pitch_cmd * np.cos(yaw_cur) - roll_cmd * np.sin(yaw_cur)

        # -------------------------- PD Controller End ---------------------------- #

        # Update old parameters with new values
        self.x_old = x_cur;
        self.x_err_old = x_err
        self.y_old = y_cur;
        self.y_err_old = y_err
        self.z_old = z_cur;
        self.z_err_old = z_err
        self.yaw_old = yaw_cur;
        self.yaw_err_old = yaw_err
        self.z_dot_old = z_dot

        return [roll_cmd_body, pitch_cmd_body, z_dot_cmd, yaw_dot_cmd]

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