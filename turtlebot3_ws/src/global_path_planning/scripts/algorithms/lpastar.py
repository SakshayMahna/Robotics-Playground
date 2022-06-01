#! /usr/bin/env python

import rospy
from math import sqrt
from algorithms.neighbors import find_weighted_neighbors

def euclidean_distance(index, goal_index, width):
  """ Heuristic Function for A Star algorithm"""
  index_x = index % width
  index_y = int(index / width)
  goal_x = goal_index % width
  goal_y = int(goal_index / width)

  distance = (index_x - goal_x) ** 2 + (index_y - goal_y) ** 2
  return sqrt(distance)

def calculate_key(index, goal_index, width, g_costs, rhs_costs):
    t1 = min(g_costs[index], rhs_costs[index]) 
    t2 = t1 + euclidean_distance(index, goal_index, width)

    return (t2, t1)

def initialize(start_index, goal_index, width, costmap):
    # create an open_list
    open_list = []

    # dict for mapping g costs (travel costs) to nodes
    g_costs = dict()

    # dict for mapping rhs costs
    rhs_costs = dict()

    # set the start's node g_cost and f_cost
    for i in range(len(costmap)):
        g_costs[i] = float('inf')
        rhs_costs[i] = float('inf')

    g_costs[start_index] = float('inf')
    rhs_costs[start_index] = 0

    g_costs[goal_index] = float('inf')
    rhs_costs[goal_index] = float('inf')

    # add start node to open list
    start_cost = calculate_key(start_index, goal_index, width, g_costs, rhs_costs)
    open_list.append([start_index, start_cost])

    shortest_path = []

    return open_list, g_costs, rhs_costs, shortest_path

def update_vertex(index, start_index, goal_index, open_list, rhs_costs, g_costs,
                  width, height, costmap, resolution):
    if index != start_index:
        min_neighbor_cost = float('inf')
        for n, c in find_weighted_neighbors(index, width, height, costmap, resolution):
            if min_neighbor_cost > g_costs[n] + costmap[index] / 255:
                min_neighbor_cost = g_costs[n] + costmap[index] / 255

        rhs_costs[index] = min_neighbor_cost
    
    # Check if index is in open_list
    in_open_list = False
    open_list_idx = -1
    for idx, element in enumerate(open_list):
        if element[0] == index:
            in_open_list = True
            open_list_idx = idx
            break
    
    if in_open_list:
        open_list.pop(open_list_idx)
        
    if g_costs[index] != rhs_costs[index]:
        open_list.append([index, calculate_key(index, goal_index, width, g_costs, rhs_costs)])
    

def compute_shortest_path(start_index, goal_index, open_list, rhs_costs, g_costs, 
                          width, height, costmap, resolution):

    if len(open_list) == 0:
        open_list.append([-1, (float('inf'), float('inf'))])
    open_list.sort(key = lambda x: x[1])

    while (open_list[0][1] < calculate_key(goal_index, goal_index, width, g_costs, rhs_costs)) or\
          (rhs_costs[goal_index] != g_costs[goal_index]):

        open_list.sort(key = lambda x: x[1])
        current_index = open_list.pop(0)[0]
        
        if g_costs[current_index] > rhs_costs[current_index]:
            g_costs[current_index] = rhs_costs[current_index]
        else:
            g_costs[current_index] = float('inf')
            update_vertex(current_index, start_index, goal_index, open_list, rhs_costs, g_costs, 
                          width, height, costmap, resolution)
                          
        neighbors = find_weighted_neighbors(current_index, width, height, costmap, resolution)
        for n, c in neighbors:
            update_vertex(n, start_index, goal_index, open_list, rhs_costs, g_costs, 
                            width, height, costmap, resolution)


def lpastar(start_index, goal_index, width, height, costmap, resolution, origin, grid_viz, previous_plan_variables):  
    open_list, g_costs, rhs_costs = [], [], []
    
    if previous_plan_variables is None:
        open_list, g_costs, rhs_costs, shortest_path = initialize(start_index, goal_index, width, costmap)
        rospy.loginfo('LPA Star: Done with initialization')

    else:
        open_list = previous_plan_variables['open_list']
        g_costs = previous_plan_variables['g_costs']
        rhs_costs = previous_plan_variables['rhs_costs']
        previous_costmap = previous_plan_variables['costmap']
        shortest_path = previous_plan_variables['shortest_path']
        start_index = previous_plan_variables['start_index']

        changed = False
        costmap = list(costmap)
        for i, index in enumerate(shortest_path):
            if costmap[index] != previous_costmap[index]:
                grid_viz.set_color(index,'orange')
                changed = True

            if changed:
                update_vertex(index, start_index, goal_index, open_list, rhs_costs, g_costs, 
                              width, height, costmap, resolution)
                for n, c in find_weighted_neighbors(index, width, height, costmap, resolution):
                    update_vertex(n, start_index, goal_index, open_list, rhs_costs, g_costs, 
                                  width, height, costmap, resolution)

    compute_shortest_path(start_index, goal_index, open_list, rhs_costs, g_costs, 
                          width, height, costmap, resolution)
    rospy.loginfo('LPA Star computed shortest path')

    # Reconstruct path by working backwards from target
    shortest_path = []
    node = goal_index
    while node != start_index:
        shortest_path.append(node)
        # get next node
        min_neighbor_cost = float('inf')
        min_neighbor_index = 0
        for n, c in find_weighted_neighbors(node, width, height, costmap, resolution):
            if n in g_costs and min_neighbor_cost > g_costs[n] + costmap[node] and n not in shortest_path:
                min_neighbor_cost = g_costs[n] + costmap[node]
                min_neighbor_index = n
        
        node = min_neighbor_index

    # reverse list
    shortest_path = shortest_path[::-1]
    rospy.loginfo('LPAStar: Done reconstructing path')

    previous_plan_variables = {
        'open_list': open_list,
        'g_costs': g_costs,
        'rhs_costs': rhs_costs,
        'costmap': costmap,
        'shortest_path': shortest_path,
        'start_index': start_index
    }

    return shortest_path, previous_plan_variables