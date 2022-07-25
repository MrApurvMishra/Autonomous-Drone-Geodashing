"""
    Saves detected targets and corresponding pose data into a JSON file.
    (for AER1217 project - 2022)
"""

#  import required libraries
import cv2
import numpy as np
import os
from matplotlib import pyplot as plt
import json

# variables' definitions for directories
data_dir = 'projectdata_images_pose/'
save_dir = 'projectdata_detected_obstacles/'
pose_file = data_dir + 'pose_info.json'
data_pose = json.load(open(pose_file))
image_list = os.listdir(data_dir)

# storing pose and detection information
obstacle_detection = {}

# Hough circle transform for finding camera centres
for img_name in image_list:
    if img_name.endswith('.jpg'):
        img = cv2.imread(data_dir + img_name)
        img_red = img[:, :, 2]                                              # extract the red channel
        img_red = cv2.threshold(img_red, 250, 255, cv2.THRESH_BINARY)[1]    # convert to binary image
        img_red = cv2.blur(img_red, (5, 5))                                 # add blur using averaging filter

        # detecting circles
        circles = cv2.HoughCircles(img_red, cv2.HOUGH_GRADIENT, 1, minDist=50,
                                   param1=40, param2=25, minRadius=25, maxRadius=100)

        # getting pose data for this frame
        img_name_json, _ = os.path.splitext(img_name)
        pose_data = data_pose[img_name_json]

        if type(circles) is np.ndarray:
            # combined pose and detection data
            obstacle_detection[img_name_json] = {}
            obstacle_detection[img_name_json]['pose'] = {}
            obstacle_detection[img_name_json]['pose']['x'] = pose_data['x']
            obstacle_detection[img_name_json]['pose']['y'] = pose_data['y']
            obstacle_detection[img_name_json]['pose']['z'] = pose_data['z']
            obstacle_detection[img_name_json]['pose']['roll'] = pose_data['roll']
            obstacle_detection[img_name_json]['pose']['pitch'] = pose_data['pitch']
            obstacle_detection[img_name_json]['pose']['yaw'] = pose_data['yaw']
            obstacle_detection[img_name_json]['circles'] = {}

            # mark circles in the images and save data to dictionary
            circles = np.uint16(np.around(circles))
            count = 0
            for i in circles[0, :]:
                count += 1  # to keep numbers of circles stored as key
                cv2.circle(img, (i[0], i[1]), i[2], (255, 0, 0), 2)
                cv2.circle(img, (i[0], i[1]), 2, (0, 255, 0), 2)
                obstacle_detection[img_name_json]['circles'][count] = [int(i[0]), int(i[1]), int(i[2])]  # (x, y, radius)

            # save the image
            cv2.imwrite(save_dir + img_name, img)

# saving the target data and pose data
with open(save_dir + 'obstacle_detection_info.json', 'w') as outfile:
    json.dump(obstacle_detection, outfile)
