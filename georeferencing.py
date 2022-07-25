import numpy as np
from math import atan2
import matplotlib.pyplot as plt
import os
import cv2
import json
import math
from matplotlib.colors import ListedColormap
from sklearn.cluster import DBSCAN
from sklearn.preprocessing import StandardScaler

# add data labels in final plot
def add_text(x, y, r):
    for k in range(len(x)):
        pos_label = '{' + str(round(x[k], 2)) + ', ' + str(round(y[k], 2)) + ', ' + str(round(r[k], 2)) + '}'
        plt.text(x[k], y[k], pos_label, horizontalalignment='center', verticalalignment='bottom')


# camera's field of view angle and aspect ratio
d_fov = 64 * np.pi / 180
AR_x = 16
AR_y = 9

# opening JSON file
detected_obstacles = open('projectdata_detected_obstacles/obstacle_detection_info.json')
detected_landmarks = open('projectdata_detected_landmarks/landmark_detection_info.json')

# load JSON object as a dictionary
obstacles_info = json.load(detected_obstacles)
landmarks_info = json.load(detected_landmarks)

# camera matrices
# body-to-camera transformation; camera parameters
body_to_camera = np.array([[ 0.0, -1.0,  0.0,    0.0],
                           [-1.0,  0.0,  0.0, 0.0125],
                           [ 0.0,  0.0, -1.0, -0.025],
                           [ 0.0,  0.0,  0.0,    1.0]])
# body_to_camera = np.array([[1.0, 0.0, 0.0, 0.0],
#                            [0.0, 1.0, 0.0, 0.0],
#                            [0.0, 0.0, 1.0, 0.0],
#                            [0.0, 0.0, 0.0, 1.0]])
K = np.array([[604.62, 0.0, 320.5],
              [0.0, 604.62, 180.5],
              [0.0, 0.0, 1.0]])

# body frame to camera frame
pos_b = np.array([0, 0, 0, 1]).reshape(4, 1)
pos_c = np.matmul(body_to_camera, pos_b)

# normalized coordinates
# pos_n = np.array([pos_c[0]/pos_c[2], pos_c[1]/pos_c[2], pos_c[2]/pos_c[2]]).reshape(3, 1)
pos_n = np.array([pos_c[0:3]/pos_c[2]]).reshape(3, 1)

# vehicle origin in image frame from bottom-left exis
pos_i = K @ pos_n
# pos_i = P @ pos_b
x_v = int(pos_i[0])
y_v = int(pos_i[1])

# extracting frames and calculating circle center coordinates in world frame
circles_x = []
circles_y = []
circles_r = []

# iterate through all detected circles
for frame in obstacles_info:
    # extracting pose and circle information
    pose = obstacles_info[frame]['pose']
    circles = obstacles_info[frame]['circles']

    for n_circle in circles:

        # extracting circle parameters: [x, y, radius]
        circle = circles[n_circle]

        # distance and angle between vehicle and circle centers in image frame
        circle_x = circle[0]
        circle_y = circle[1]
        dist_pixel = math.sqrt((x_v - circle_x) ** 2 + (y_v - circle_y) ** 2)
        angle = atan2(circle_y - y_v, circle_x - x_v)

        # circle center coordinates in world frame
        if dist_pixel <= 275:
            height = pose['z']
            diagonal = 2 * np.tan(d_fov / 2) * height
            ratio_const = np.sqrt((diagonal ** 2) / (AR_x ** 2 + AR_y ** 2))
            width = AR_x * ratio_const
            m_per_pixel = width / 640
            dist_m = dist_pixel * m_per_pixel

            # circle centres in world frame
            c_x_w = pose['x'] + dist_m * np.cos(angle + pose['yaw'])
            c_y_w = pose['y'] + dist_m * np.sin(angle + pose['yaw'])

            circles_x.append(c_x_w)
            circles_y.append(c_y_w)
            circles_r.append(circle[2] * m_per_pixel * 4)

# DBSCAN - density-based clustering
# 2D features
features = [circles_x, circles_y]
features = np.asarray(features)
features = np.transpose(features)

# scaling data for clustering, removes mean and scales to unit variance
scaler = StandardScaler()
scaled = scaler.fit_transform(features)

# segregate the data into clusters
dbscan = DBSCAN(eps=0.47, min_samples=4)
clusters = dbscan.fit_predict(scaled)

# extracting landmark positions
landmark1 = []
landmark2 = []
landmark3 = []
landmark4 = []

# iterate through all detected landamrks
for frame in landmarks_info:

    # extracting pose and circle information
    pose = landmarks_info[frame]['pose']
    landmarks = landmarks_info[frame]['landmarks']

    for n_landmark in landmarks:

        # extracting landmark parameters: [x, y, angle]
        landmark = landmarks[n_landmark]

        # distance and angle between vehicle and circle centers in image frame
        landmark_x = landmark[0]
        landmark_y = landmark[1]
        landmark_a = landmark[2] + pose['yaw'] - np.pi/2
        dist_pixel = math.sqrt((x_v - landmark_x) ** 2 + (y_v - landmark_y) ** 2)
        angle = atan2(landmark_y - y_v, landmark_y - x_v)

        # circle center coordinates in world frame
        # if dist_pixel <= 125:
        if dist_pixel <= 500:
            height = pose['z']
            diagonal = 2 * np.tan(d_fov / 2) * height
            ratio_const = np.sqrt((diagonal ** 2) / (AR_x ** 2 + AR_y ** 2))
            width = AR_x * ratio_const
            m_per_pixel = width / 640
            dist_m = dist_pixel * m_per_pixel

            # circle centres in world frame
            landmark_x_w = pose['x'] + dist_m * np.cos(angle + pose['yaw'])
            landmark_y_w = pose['y'] + dist_m * np.sin(angle + pose['yaw'])

            if n_landmark == '1':
                landmark1.append([landmark_x_w, landmark_y_w, landmark_a])
            elif n_landmark == '2':
                landmark2.append([landmark_x_w, landmark_y_w, landmark_a])
            elif n_landmark == '3':
                landmark3.append([landmark_x_w, landmark_y_w, landmark_a])
            elif n_landmark == '4':
                landmark4.append([landmark_x_w, landmark_y_w, landmark_a])

# plotting the circles detected
plt.rcParams.update({'font.size': 10})
plt.figure(1)
plt.scatter(circles_x, circles_y, s=2, alpha=0.5)
plt.title("Circles detected in the inertial frame")
plt.xlabel("x-coordinates in metres")
plt.ylabel("y-coordinates in metres")
plt.grid()

# Plot the clusters formed
colormap = ListedColormap(["black", "lightgrey", "blue", "red", "green", "darkorange", "cyan", "magenta"])
plt.figure(2)
plt.scatter(features[:, 0], features[:, 1], c=clusters, cmap=colormap, marker='o', edgecolors='k')
plt.title("6 clusters of circles' positions, outliers in grey")
plt.xlabel("x-coordinates in metres")
plt.ylabel("y-coordinates in metres")
plt.grid()

# calculating mean and standard deviation of each circle
centres_x = []
centres_y = []
obs_radii = []
n_clusters = len(set(clusters)) - (1 if -1 in clusters else 0)
for i in range(n_clusters):
    x_coord = [circles_x[x] for x in range(len(clusters)) if clusters[x] == i]
    y_coord = [circles_y[y] for y in range(len(clusters)) if clusters[y] == i]
    radius  = [circles_r[x] for x in range(len(clusters)) if clusters[x] == i]
    centres_x.append(np.mean(x_coord))
    centres_y.append(np.mean(y_coord))
    obs_radii.append(np.mean(radius))

# averaging the landmark positions
landmark1 = np.mean(np.array(landmark1), axis=0)
landmark2 = np.mean(np.array(landmark2), axis=0)
landmark3 = np.mean(np.array(landmark3), axis=0)
landmark4 = np.mean(np.array(landmark4), axis=0)

# plotting the calculated circle positions
plt.figure(3)
plt.plot(centres_x, centres_y, 'or', markersize=15)
plt.plot(landmark1[0], landmark1[1], 'sg', markersize=15)
plt.plot(landmark2[0], landmark2[1], 'sg', markersize=15)
plt.plot(landmark3[0], landmark3[1], 'sg', markersize=15)
plt.plot(landmark4[0], landmark4[1], 'sg', markersize=15)
add_text(centres_x, centres_y, obs_radii)
add_text([landmark1[0]], [landmark1[1]], [landmark1[2]])
add_text([landmark2[0]], [landmark2[1]], [landmark2[2]])
add_text([landmark3[0]], [landmark3[1]], [landmark3[2]])
add_text([landmark4[0]], [landmark4[1]], [landmark4[2]])
plt.text(landmark1[0], landmark1[1], 'Casa Loma', horizontalalignment='center', verticalalignment='top')
plt.text(landmark2[0], landmark2[1], 'CN Tower', horizontalalignment='center', verticalalignment='top')
plt.text(landmark3[0], landmark3[1], 'Nathan Philips', horizontalalignment='center', verticalalignment='top')
plt.text(landmark4[0], landmark4[1], 'Princes Gates', horizontalalignment='center', verticalalignment='top')

plt.xlim(0, 9)
plt.ylim(0, 9)
plt.title("Position of the circle markers in the inertial frame")
plt.xlabel("x-coordinates in metres")
plt.ylabel("y-coordinates in metres")
plt.grid()
plt.show()

# closing JSON file
detected_obstacles.close()
detected_landmarks.close()
