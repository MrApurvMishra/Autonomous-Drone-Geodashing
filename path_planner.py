
# About: Dijkstra's implementation for AER1217 Project <br>
# Date Started: April 20, 2022

import numpy as np
import json
import math
from operator import itemgetter
import time
import pickle
import matplotlib.pyplot as plt

## user-defined variables [changes ONLY here] 
target_seq          = [1, 4, 3, 1]
start_pos           = [1, 1]        # in metres from inertial reference frame
map_filepath        = 'occupancy_grid.npy'
targetdata_filepath = 'target_data.json'


## reading data
map     = np.load(map_filepath)
map_dim = map.shape[0] 
with open(targetdata_filepath, 'r') as f:
  targetdata = json.load(f)


## helper functions
def find_neighbours(map, node):
    """returns valid list of neighbours for a node/cell in the map"""

    # node to find neighbours of
    x,y = node[0], node[1]

    # potential neighbour points
    n1 = [x+1, y];        n5 = [x-1, y]
    n2 = [x+1, y+1];      n6 = [x-1, y-1]
    n3 = [x, y+1];        n7 = [x, y-1]
    n4 = [x-1, y+1];      n8 = [x+1, y-1]
    n_list       = [n1, n2, n3, n4, n5, n6, n7, n8]
    final_n_list = []

    # map's height/width
    map_hgt_wdh = map.shape[0]

    for n in n_list:
        x_matrix_frame = map_hgt_wdh-1-n[1]
        y_matrix_frame = n[0]
        if 0<=x_matrix_frame<map_hgt_wdh and 0<=y_matrix_frame<map_hgt_wdh:
            if map[x_matrix_frame][y_matrix_frame]==0:
                final_n_list.append(n)

    return final_n_list


def euclidean_distance(node1, node2):
    """returns the euclidean distance between two nodes in the map"""

    return round(math.sqrt((node1[0]-node2[0])**2 + (node1[1]-node2[1])**2),2)

def node_id(node, map_dim):
    """returns a unique integer id of a node position (x,y) and map size"""

    return (node[1]%map_dim)*map_dim+node[0] 

def node_pos(node_id, map_dim):
    """returns the x, y position of the node given its node id and map size"""

    return [node_id%map_dim, node_id//map_dim]

def dijkstra(target_seq, targetdata, start_pos):
    """
    returns the final shortest path using dijkstra's algorithm
    
    Args:
        target_seq List[int]: the sequence of targets that need to be traversed
        targetdata {}: dictionary data containing landmark pose information
        start_pos [float, float]: starting position of the drone

    Return:
        global_final_path List[List[float, float]]: complete path to traverse
    """
    global_final_path = []

    for i in range(len(target_seq)+1):
        if i==0:                     # drone's start position is the start location
            start = [int(round(start_pos[0]*10)), int(round(start_pos[1]*10))]
            
            lmark_num        = target_seq[i]
            lmark_data       = targetdata['landmark_' + str(lmark_num)]['pose']
            lmark_x, lmark_y = lmark_data[0], lmark_data[1]
            goal             = [int(round(lmark_x*10)), int(round(lmark_y*10))]
        elif i<len(target_seq):      # previous landmark is the start location
            lmark_num        = target_seq[i-1]
            lmark_data       = targetdata['landmark_' + str(lmark_num)]['pose']
            lmark_x, lmark_y = lmark_data[0], lmark_data[1]
            start            = [int(round(lmark_x*10)), int(round(lmark_y*10))]

            lmark_num        = target_seq[i]
            lmark_data       = targetdata['landmark_' + str(lmark_num)]['pose']
            lmark_x, lmark_y = lmark_data[0], lmark_data[1]
            goal             = [int(round(lmark_x*10)), int(round(lmark_y*10))]
        else:                        # going back to start position
            lmark_num        = target_seq[i-1]
            lmark_data       = targetdata['landmark_' + str(lmark_num)]['pose']
            lmark_x, lmark_y = lmark_data[0], lmark_data[1]
            start            = [int(round(lmark_x*10)), int(round(lmark_y*10))]

            goal             = [int(round(start_pos[0]*10)), int(round(start_pos[1]*10))]

        # core dijkstra's algorithm
        visited        = [start]          # list for visited nodes
        pr_queue       = [[start, 0]]     # priority queue, the second element is cost-to-come
        backtrack_dict = {}               # {node: [parent_node, cost_to_come]}, needed for backtracking

        while pr_queue:
            # access the top element in the priority queue
            top_element  = pr_queue[0]
            parent_node  = top_element[0]
            cost_to_come = top_element[1]

            # remove top element
            pr_queue     = pr_queue[1:]

            # goal reached
            if parent_node==goal:
                break

            # find neighbours for the current node
            neighbour_list = find_neighbours(map, parent_node)

            # traverse all the neighbours that are not already visited
            for neighbour in neighbour_list:
                if neighbour not in visited:

                    # node added to visited set
                    visited.append(neighbour)

                    # find the total cost to come for the current neighbour node    
                    tot_cost_to_come = cost_to_come + euclidean_distance(parent_node, neighbour)

                    # update the backtracking information
                    backtrack_dict[node_id(neighbour, map_dim)] = \
                        [node_id(parent_node, map_dim), tot_cost_to_come]

                    # append the node to priority queue
                    pr_queue.append([neighbour, tot_cost_to_come])


            pr_queue = sorted(pr_queue, key=itemgetter(1))

        # backtracking to get the shortest path from goal to start
        final_path    = [goal]
        cur_node_id   = node_id(goal, map_dim)
        start_node_id = node_id(start, map_dim)

        while cur_node_id!=start_node_id:
            parent_node_id = backtrack_dict[cur_node_id][0]
            final_path.append(node_pos(parent_node_id, map_dim))

            cur_node_id = parent_node_id
    
        # reversing to get path from start to goal
        final_path.reverse()

        # appending this segment to global path
        global_final_path.append(final_path)

    return global_final_path 
        

## run dijkstra
# start time
s_time = time.time()
complete_global_path = dijkstra(target_seq, targetdata, start_pos)
print('Time taken by dijkstra to find the path (seconds): ', (time.time()-s_time))


## save final path and plot
fig_grid = pickle.load(open('map_figure.pickle','rb'))

# adding start position on the map
start = [int(round(start_pos[0]*10)), int(round(start_pos[1]*10))]
plt.scatter(start[0], map_dim-1-start[1], 40, marker="h")
plt.text(start[0], map_dim-1-start[1], 'start', color='white')

# final path to save
inertial_complete_global_path = []

for segment in complete_global_path:
    inertial_path = [[pos[0]/10, pos[1]/10] for pos in segment]
    inertial_complete_global_path.append(inertial_path)

    x_pos = [pos[0] for pos in segment]
    y_pos = [map_dim-1-pos[1] for pos in segment]
    plt.plot(x_pos, y_pos, 'w')

# np.save('final_path', np.array(inertial_complete_global_path), allow_pickle=True, fix_imports=True)
# np.save('target_sequence', np.array(target_seq), allow_pickle=True, fix_imports=True)

pickle.dump(np.array(inertial_complete_global_path),open('final_path.pickle','wb'), protocol=2)
pickle.dump(np.array(target_seq),open('target_sequence.pickle','wb'), protocol=2)

plt.title('Path for sequence: ' + str(target_seq[0]) + '-' + str(target_seq[1]) + '-' \
    + str(target_seq[2]) + '-' + str(target_seq[3]))
plt.savefig('final_path_' + str(target_seq[0]) + '-' + str(target_seq[1]) + '-' \
    + str(target_seq[2]) + '-' + str(target_seq[3]) + '.png')



