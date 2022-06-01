#! /usr/bin/env python

"""
Dijkstra's algorithm path planning exercise solution
Author: Roberto Zegers R.
Copyright: Copyright (c) 2020, Roberto Zegers R.
License: BSD-3-Clause
Date: Nov 30, 2020
Usage: roslaunch unit2_pp unit2_solution.launch
"""

import rospy
from algorithms.neighbors import find_neighbors

def dijkstra(start_index, goal_index, width, height, costmap, resolution, origin, grid_viz, previous_plan_variables):
  ''' 
  Performs Dijkstra's shortes path algorithm search on a costmap with a given start and goal node
  '''

  # create an open_list
  open_list = []

  # set to hold already processed nodes
  closed_list = set()

  # dict for mapping children to parent
  parents = dict()

  # dict for mapping g costs (travel costs) to nodes
  g_costs = dict()

  # set the start's node g_cost
  g_costs[start_index] = 0

  # add start node to open list
  open_list.append([start_index, 0])

  shortest_path = []

  path_found = False
  rospy.loginfo('Dijkstra: Done with initialization')

  # Main loop, executes as long as there are still nodes inside open_list
  while open_list:

    # sort open_list according to the lowest 'g_cost' value (second element of each sublist)
    open_list.sort(key = lambda x: x[1]) 
    # extract the first element (the one with the lowest 'g_cost' value)
    current_node = open_list.pop(0)[0]

    # Close current_node to prevent from visting it again
    closed_list.add(current_node)

    # Optional: visualize closed nodes
    grid_viz.set_color(current_node,"pale yellow")

    # If current_node is the goal, exit the main loop
    if current_node == goal_index:
      path_found = True
      break

    # Get neighbors of current_node
    neighbors = find_neighbors(current_node, width, height, costmap, resolution)

    # Loop neighbors
    for neighbor_index, step_cost in neighbors:

      # Check if the neighbor has already been visited
      if neighbor_index in closed_list:
        continue

      # calculate g_cost of neighbour considering it is reached through current_node
      g_cost = g_costs[current_node] + step_cost

      # Check if the neighbor is in open_list
      in_open_list = False
      for idx, element in enumerate(open_list):
        if element[0] == neighbor_index:
          in_open_list = True
          break

      # CASE 1: neighbor already in open_list
      if in_open_list:
        if g_cost < g_costs[neighbor_index]:
          # Update the node's g_cost inside g_costs
          g_costs[neighbor_index] = g_cost
          parents[neighbor_index] = current_node
          # Update the node's g_cost inside open_list
          open_list[idx] = [neighbor_index, g_cost]

      # CASE 2: neighbor not in open_list
      else:
        # Set the node's g_cost inside g_costs
        g_costs[neighbor_index] = g_cost
        parents[neighbor_index] = current_node
        # Add neighbor to open_list
        open_list.append([neighbor_index, g_cost])

        # Optional: visualize frontier
        grid_viz.set_color(neighbor_index,'orange')

  rospy.loginfo('Dijkstra: Done traversing nodes in open_list')

  if not path_found:
    rospy.logwarn('Dijkstra: No path found!')
    return shortest_path

  # Reconstruct path by working backwards from target
  if path_found:
      node = goal_index
      shortest_path.append(goal_index)
      while node != start_index:
          shortest_path.append(node)
          # get next node
          node = parents[node]
  # reverse list
  shortest_path = shortest_path[::-1]
  rospy.loginfo('Dijkstra: Done reconstructing path')

  return shortest_path, None

