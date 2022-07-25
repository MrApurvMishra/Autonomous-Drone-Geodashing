"""
About: Saves the obstacle and landmark information to json file [AER1217]
Date : April 15, 2022
"""

import json

# dictionary to contain all poses
pose_dict = {
    "obstacle_0": {
        "pose": [1.23, 2.46, 0.00, 0.00, -0.00, 0.00],
        "scale": [1.87, 1.87, 1000.0],
    },
    "obstacle_1": {
        "pose": [1.8, 4.43, 0.00, 0.00, -0.00, 0.00],
        "scale": [1.2, 1.2, 1000.0],
    },
    "obstacle_2": {
        "pose": [3.27, 7.85, 0.00, 0.00, -0.00, 0.00],
        "scale": [1.17, 1.17, 1000.0],
    },
    "obstacle_3": {
        "pose": [4.89, 2.78, 0.00, 0.00, -0.00, 0.00],
        "scale": [1.55, 1.55, 1000.0],
    },
    "obstacle_4": {
        "pose": [5.92, 5.36, 0.00, 0.00, -0.00, 0.00],
        "scale": [0.9, 0.9, 1000.0],
    },
    "obstacle_5": {
        "pose": [6.39, 3.58, 0.00, 0.00, -0.00, 0.00],
        "scale": [0.65, 0.65, 1000.0],
    },
    "obstacle_6": {
        "pose": [7.32, 2.12, 0.00, 0.00, -0.00, 0.00],
        "scale": [0.74, 0.74, 1000.0],
    },
    # Casa Loma
    "landmark_1": {
        "pose": [5.82, 6.74, 0.00, 0.00, 0.00, 0.6],
        "scale": [1.0, 1.0, 1.0],
    },
    # CN Tower
    "landmark_2": {
        "pose": [2.23, 3.52, 0.00, 0.00, 0.00, -1.34],
        "scale": [1.0, 1.0, 1.0],
    },
    # Nathan Philips Square
    "landmark_3": {
        "pose": [1.14, 6.98, 0.00, 0.00, -0.00, -3.37],
        "scale": [1.0, 1.0, 1.0],
    },
    # Princes Gates
    "landmark_4": {
        "pose": [6.94, 5.41, 0.00, 0.00, 0.00, -0.5],
        "scale": [1.0, 1.0, 1.0],
    }
}

with open('target_data.json', 'w') as fp:
    json.dump(pose_dict, fp, indent=4)