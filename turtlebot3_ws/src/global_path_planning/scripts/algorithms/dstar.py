#! /usr/bin/env python

import rospy
from math import sqrt
from algorithms.neighbors import find_weighted_neighbors

def euclidean_distance(index, start_index, width):
  """ Heuristic Function for A Star algorithm"""
  index_x = index % width
  index_y = int(index / width)
  start_x = start_index % width
  start_y = int(start_index / width)

  distance = (index_x - start_x) ** 2 + (index_y - start_y) ** 2
  return sqrt(distance)

def calculate_key(index, km, start_index, width, g_costs, rhs_costs):
    t1 = min(g_costs[index], rhs_costs[index]) 
    t2 = t1 + euclidean_distance(index, start_index, width) + km

    return (t2, t1)

def initialize(start_index, goal_index, width):
    # create an open_list
    open_list = []

    # km variable
    km = 0

    # dict for mapping g costs (travel costs) to nodes
    g_costs = dict()

    # dict for mapping rhs costs
    rhs_costs = dict()

    # set the goal node's g_cost and f_cost
    g_costs[goal_index] = float('inf')
    rhs_costs[goal_index] = 0

    # add start node to open list
    start_cost = [euclidean_distance(goal_index, start_index), 0]
    open_list.append([start_index, start_cost])

    shortest_path = []

    return open_list, km, g_costs, rhs_costs, shortest_path

def update_vertex(index, km, start_index, goal_index, open_list, rhs_costs, g_costs,
                  width, height, costmap, resolution):
    if index not in g_costs:
        g_costs[index] = float('inf')
    if index not in rhs_costs:
        rhs_costs[index] = float('inf')

    # Check if index is in open_list
    in_open_list = False
    open_list_idx = -1
    for idx, element in enumerate(open_list):
        if element[0] == index:
            in_open_list = True
            open_list_idx = idx
            break

    if g_costs[index] != rhs_costs[index] and in_open_list:
        open_list[open_list_idx][1] = calculate_key(index, km, start_index, width, g_costs, rhs_costs)
    
    elif g_costs[index] != rhs_costs[index] and not in_open_list:
        open_list.append([index, calculate_key(index, km, start_index, width, g_costs, rhs_costs)])
    
    elif g_costs[index] == rhs_costs[index] and in_open_list:
        open_list.pop(open_list_idx)
    

def compute_shortest_path(start_index, goal_index, km, open_list, rhs_costs, g_costs, 
                          width, height, costmap, resolution):
    open_list.sort(key = lambda x: x[1])

    while (open_list[0][1] < calculate_key(start_index, km, start_index, width, g_costs, rhs_costs)) or\
          (rhs_costs[start_index] != g_costs[start_index]):

        open_list.sort(key = lambda x: x[1])
        current_index = open_list[0][0]
        k_old = open_list[0][1]
        k_new = calculate_key(open_list[0][0], km, start_index, width, g_costs, rhs_costs)

        if k_old < k_new:
            open_list[0][1] = k_new

        if current_index not in g_costs:
            g_costs[current_index] = float('inf')
        if current_index not in rhs_costs:
            rhs_costs[current_index] = float('inf')
        
        elif g_costs[current_index] > rhs_costs[current_index]:
            g_costs[current_index] = rhs_costs[current_index]
            open_list.pop(0)
            neighbors = find_weighted_neighbors(current_index, width, height, costmap, resolution)
            for n, c in neighbors:
                if n != goal_index:
                    rhs_costs[n] = min(rhs_costs[n], g_costs[n] + c)

                update_vertex(n, start_index, goal_index, open_list, rhs_costs, g_costs, 
                              width, height, costmap, resolution)

        else:
            g_old = g_costs[current_index]
            g_costs[current_index] = float('inf')
            neighbors = find_weighted_neighbors(current_index, width, height, costmap, resolution)
            if rhs_costs[current_index] == g_old and current_index != goal_index:

                rhs_costs[current_index] = min(rhs_costs[current_index])


def dstar(start_index, goal_index, width, height, costmap, resolution, origin, grid_viz, previous_plan_variables):  
    open_list, g_costs, rhs_costs = [], [], []
    
    if previous_plan_variables is None:
        open_list, km, g_costs, rhs_costs, shortest_path = initialize(start_index, goal_index, width)
        rospy.loginfo('LPA Star: Done with initialization')

    else:
        open_list = previous_plan_variables['open_list']
        g_costs = previous_plan_variables['g_costs']
        rhs_costs = previous_plan_variables['rhs_costs']
        previous_costmap = previous_plan_variables['costmap']
        shortest_path = previous_plan_variables['shortest_path']
        start_index = previous_plan_variables['start_index']
        
        for i, index in enumerate(shortest_path):
            if costmap[index] != previous_costmap[index]:
                print((i, index, costmap[index], previous_costmap[index]))
                update_vertex(index, start_index, goal_index, open_list, rhs_costs, g_costs, 
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
            if n in g_costs and min_neighbor_cost > g_costs[n] + c and n not in shortest_path:
                min_neighbor_cost = g_costs[n] + c
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