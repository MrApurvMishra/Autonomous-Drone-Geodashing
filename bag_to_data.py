#!/usr/bin/env python2

"""ROS Node for fetching time-synchronized image and pose data from bag file"""

from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
from geometry_msgs.msg import TransformStamped
from tf.transformations import euler_from_quaternion
import rospy
import cv2
import json
import numpy as np

class GetDataFromBag(object):
    """ROS Node for fetching time-synchronized image and pose data from bag file"""

    def __init__(self):
        """Initialize the GetDataFromBag class"""

        # Subscribers
        self.sub_image_data = rospy.Subscriber('/ardrone/bottom/image_raw',
                                               Image,
                                               self.save_image_data)

        self.sub_vicon_data = rospy.Subscriber('/vicon/ARDroneCarre/ARDroneCarre',
                                               TransformStamped,
                                               self.current_pose)

        # for image data
        self.bridge       = CvBridge()
        self.img_save_dir = '/home/rob521_3/aer1217/bagfiles/projectdata_images_pose/'
        self.image_id     = 1

        # for pose data
        self.x = 0.0;     self.roll  = 0.0
        self.y = 0.0;     self.pitch = 0.0
        self.z = 0.0;     self.yaw   = 0.0  
        self.pose_info_dict = {}   


    def save_image_data(self, image_msg):
        """saves image data from the image topic being published"""

        try:
            cv_image = self.bridge.imgmsg_to_cv2(image_msg, 'bgr8')
            image_name = 'frame_' + str(self.image_id)
        except CvBridgeError as e:
            print(e)

        # storing the pose information for current timestamp
        self.pose_info_dict[image_name]          = {}
        self.pose_info_dict[image_name]['x']     = self.x
        self.pose_info_dict[image_name]['y']     = self.y
        self.pose_info_dict[image_name]['z']     = self.z
        self.pose_info_dict[image_name]['roll']  = self.roll
        self.pose_info_dict[image_name]['pitch'] = self.pitch
        self.pose_info_dict[image_name]['yaw']   = self.yaw
        
        # save image
        cv2.imwrite(self.img_save_dir + image_name + '.jpg', cv_image)

        # increase frame_id counter for next image
        self.image_id += 1


    def current_pose(self, vicon_msg):
        """updates the current pose of the robot"""

        # translational variables
        self.x = vicon_msg.transform.translation.x
        self.y = vicon_msg.transform.translation.y
        self.z = vicon_msg.transform.translation.z

        # rotational variables
        quaternion = np.array([vicon_msg.transform.rotation.x,
                               vicon_msg.transform.rotation.y,
                               vicon_msg.transform.rotation.z,
                               vicon_msg.transform.rotation.w])
        euler      = euler_from_quaternion(quaternion)
        roll_cur   = euler[0]
        pitch_cur  = euler[1]
        yaw_cur    = euler[2]
        self.roll  = roll_cur
        self.pitch = pitch_cur
        self.yaw   = yaw_cur


    def write_pose_data(self):
        """writes the pose data to disk"""

        with open(self.img_save_dir + 'pose_info.json', 'w') as outfile:
            json.dump(self.pose_info_dict, outfile)
            print('Pose data written to disk!')

if __name__ == '__main__':
    rospy.init_node('bag_to_data')
    get_bag_data = GetDataFromBag()
    rospy.spin()
    get_bag_data.write_pose_data()    # save pose data to disk