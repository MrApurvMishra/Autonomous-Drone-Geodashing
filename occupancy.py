# TO GENERATE AN OCCUPANCY GRIP MAP FOR AER1217 PROJECT

# import required files
import matplotlib.pyplot as plt
import numpy as np
import json
import pickle

# limits or boundaries of the environment
x_bounds = [0, 9]
y_bounds = [0, 9]
z_bounds = [0, 3]

# reading pose data from json file
filepath = './target_data.json'
with open(filepath, 'r') as f:
  pose_dict = json.load(f)

# occupancy grid - initialization
len_x = x_bounds[1] - x_bounds[0]
len_y = y_bounds[1] - y_bounds[0]
x_len = len_x*10
y_len = len_y*10

# two occupancy grid variables and extra padding
occupancy_grid_viz  = np.zeros((x_len, y_len))
occupancy_grid_plan = np.zeros((x_len, y_len))
padding             = 2

# initialising the figure
fig_object = plt.figure(figsize=(9, 9))

# marking obstacle regions
obstacles_n = 7
for n in range(obstacles_n):
    curr_obs = "obstacle_" + str(n)
    x_coord  = int(round(pose_dict[curr_obs]['pose'][0] * 10))
    y_coord  = int(round(pose_dict[curr_obs]['pose'][1] * 10))
    # print('Obstacle coordinates: ', x_coord, y_coord)
    radius   = int(round(pose_dict[curr_obs]['scale'][0] * 10) / 2)
    plt.scatter(x_coord, x_len-1-y_coord, 20)
    plt.text(x_coord, x_len-1-y_coord, curr_obs)
    
    # updating vizualisation matrix
    for x in range(x_coord - radius, x_coord + radius):
        for y in range(y_coord - radius, y_coord + radius):
            if 0 < x < 90 and 0 < y < 90:
                dist = np.sqrt((x_coord - x) ** 2 + (y_coord - y) ** 2)
                if dist < radius:
                    occupancy_grid_viz[x_len-1-y][x] = 1

    # updating planning matrix
    for x in range(x_coord - radius - padding, x_coord + radius + padding):
        for y in range(y_coord - radius - padding, y_coord + radius + padding):
            if 0 < x < 90 and 0 < y < 90:
                dist = np.sqrt((x_coord - x) ** 2 + (y_coord - y) ** 2)
                if dist < radius + padding:
                    occupancy_grid_plan[x_len-1-y][x] = 1
            


# adding landmark locations on the plot
l1_x_coord  = int(round(pose_dict['landmark_1']['pose'][0] * 10))
l1_y_coord  = int(round(pose_dict['landmark_1']['pose'][1] * 10))
plt.scatter(l1_x_coord, x_len-1-l1_y_coord, 40, marker="*")
plt.text(l1_x_coord, x_len-1-l1_y_coord, 'Landmark 1: Casa Loma', color='red')

l2_x_coord  = int(round(pose_dict['landmark_2']['pose'][0] * 10))
l2_y_coord  = int(round(pose_dict['landmark_2']['pose'][1] * 10))
plt.scatter(l2_x_coord, x_len-1-l2_y_coord, 40, marker="*")
plt.text(l2_x_coord, x_len-1-l2_y_coord, 'Landmark 2: CN Tower', color='red')

l3_x_coord  = int(round(pose_dict['landmark_3']['pose'][0] * 10))
l3_y_coord  = int(round(pose_dict['landmark_3']['pose'][1] * 10))
plt.scatter(l3_x_coord, x_len-1-l3_y_coord, 40, marker="*")
plt.text(l3_x_coord, x_len-1-l3_y_coord, 'Landmark 3: Nathan Philips Square', color='red')

l4_x_coord  = int(round(pose_dict['landmark_4']['pose'][0] * 10))
l4_y_coord  = int(round(pose_dict['landmark_4']['pose'][1] * 10))
plt.scatter(l4_x_coord, x_len-1-l4_y_coord, 40, marker="*")
plt.text(l4_x_coord, x_len-1-l4_y_coord, 'Landmark 4: Princes Gates', color='red')


# plotting to verify
# plt.axis([0, 90, 0, 90])
plt.imshow(occupancy_grid_viz)
plt.xlabel('x position')
plt.ylabel('y position')
plt.title('Occupancy Map')
plt.savefig('occupancy_grid_90.png')
# plt.show()


# save the array and figure
pickle.dump(fig_object,open('map_figure.pickle','wb'))
np.save('occupancy_grid', occupancy_grid_plan)
