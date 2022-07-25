"""
    Using SIFT and brute-force method to match and detect targets
    (for AER1217 project - 2022)
"""

# import required libraries
import numpy as np
import cv2
import os
import matplotlib.pyplot as plt
import json
from math import atan2

# variables' definitions for directories
data_dir = 'projectdata_images_pose/'
save_dir = 'projectdata_detected_landmarks/'
pose_file = data_dir + 'pose_info.json'
data_pose = json.load(open(pose_file))
image_list = os.listdir(data_dir)

# storing pose and detection information
landmark_detection = {}

# minimum required number of matches
MIN_MATCH_COUNT = 75

# initiate SIFT detector
sift = cv2.SIFT_create()

# keypoint matching
FLANN_INDEX_KDTREE = 1
index_params = dict(algorithm=FLANN_INDEX_KDTREE, trees=5)
search_params = dict(checks=50)

# iterate through all four landmarks
for i in range(4):

    # take the landmark image as input
    if i == 0:
        img1 = cv2.imread('casa_loma.JPG')
    elif i == 1:
        img1 = cv2.imread('cn_tower.JPG')
    elif i == 2:
        img1 = cv2.imread('nathan_philips_square.JPG')
    else:
        img1 = cv2.imread('princes_gate.JPG')

    img1_gray = cv2.cvtColor(img1, cv2.COLOR_BGR2GRAY)

    # iterate through all the collected images
    for img_name in image_list:

        # check for a JPG file
        if img_name.endswith('.jpg'):

            # take the collected images as input
            img2 = cv2.imread(data_dir + img_name)
            img2_gray = cv2.cvtColor(img2, cv2.COLOR_BGR2GRAY)

            # find the keypoints and descriptors with SIFT
            kp1, des1 = sift.detectAndCompute(img1_gray, None)
            kp2, des2 = sift.detectAndCompute(img2_gray, None)

            try:
                # keypoint matching
                flann = cv2.FlannBasedMatcher(index_params, search_params)
                matches = flann.knnMatch(des1, des2, k=2)
            except:
                continue

            # store all the good matches as per Lowe's ratio test.
            good = []
            for m, n in matches:
                if m.distance < 0.5 * n.distance:
                    good.append(m)

            # find the transformation
            if len(good) > MIN_MATCH_COUNT:
                src_pts = np.float64([kp1[m.queryIdx].pt for m in good]).reshape(-1, 1, 2)
                dst_pts = np.float64([kp2[m.trainIdx].pt for m in good]).reshape(-1, 1, 2)

                # extract coordinates of the landmark [x, y]
                xy_coords = np.around(np.median(dst_pts, axis=0)[0])
                # xy_coords = np.around(np.mean(dst_pts, axis=0)[0])

                # find the transformation matrix
                M, mask = cv2.findHomography(src_pts, dst_pts, cv2.RANSAC, 5.0)
                matchesMask = mask.ravel().tolist()

                # calculate orientation from the rotation matrix
                yaw = - atan2(M[1, 0], M[0, 0])

                # find perspective transformation
                h, w = img1_gray.shape
                pts = np.float32([[0, 0], [0, h-1], [w-1, h-1], [w-1, 0]]).reshape(-1, 1, 2)    # borders
                dst = cv2.perspectiveTransform(pts, M)
                img2 = cv2.polylines(img2, [np.int32(dst)], True, 255, 3, cv2.LINE_AA)

                # visualize
                draw_params = dict(matchColor=(0, 255, 0),      # draw matches in green color
                                   singlePointColor=None,
                                   matchesMask=matchesMask,     # draw only inliers
                                   flags=2)
                img_matched = cv2.drawMatches(img1, kp1, img2, kp2, good, None, **draw_params)
                # cv2.imshow('gray', img_matched)
                # plt.show()
                print("Good enough matches found - {}/{}".format(len(good), MIN_MATCH_COUNT))

                # getting pose data for this frame
                img_name_json, _ = os.path.splitext(img_name)
                pose_data = data_pose[img_name_json]

                # combined pose and detection data
                landmark_detection[img_name_json] = {}
                landmark_detection[img_name_json]['pose'] = {}
                landmark_detection[img_name_json]['pose']['x'] = pose_data['x']
                landmark_detection[img_name_json]['pose']['y'] = pose_data['y']
                landmark_detection[img_name_json]['pose']['z'] = pose_data['z']
                landmark_detection[img_name_json]['pose']['roll'] = pose_data['roll']
                landmark_detection[img_name_json]['pose']['pitch'] = pose_data['pitch']
                landmark_detection[img_name_json]['pose']['yaw'] = pose_data['yaw']
                landmark_detection[img_name_json]['landmarks'] = {}

                # [x-coordinate, y-coordinate, angle from heading or y-axis towards top (in radians)]
                landmark_detection[img_name_json]['landmarks'][i+1] = [xy_coords[1], xy_coords[0], yaw]

                # save the image with matching features drawn, file name as <frame_xxxx-i.jpg>
                file_name = img_name_json + '-' + str(i+1)
                cv2.imwrite(save_dir + file_name + '.jpg', img_matched)

            else:
                print("Not enough matches found - {}/{}".format(len(good), MIN_MATCH_COUNT))
                matchesMask = None

# saving the target data and pose data
with open(save_dir + 'landmark_detection_info.json', 'w') as outfile:
    json.dump(landmark_detection, outfile)
