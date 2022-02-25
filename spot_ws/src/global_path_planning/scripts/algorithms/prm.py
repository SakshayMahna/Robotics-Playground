#! /usr/bin/env python

import rospy
import random
import math
import numpy as np
from scipy.spatial import KDTree

N_SAMPLE = 10
LETHAL_COST = 150
PATH_RESOLUTION = 1

def init_nodes(start_index, goal_index, width):
  start_x = int(start_index % width)
  start_y = int(start_index // width)
  goal_x = int(goal_index % width)
  goal_y = int(goal_index // width)

  x_min = min(start_x, goal_x)
  x_max = max(start_x, goal_x)
  y_min = min(start_y, goal_y)
  y_max = max(start_y, goal_y)

  index_list = []
  for x in range(x_min, x_max + 1):
    for y in range(y_min, y_max + 1):
      index_list.append(y * width + x)

  return index_list

def check_collision(index, costmap):
  """ Check Collision, if safe return True """
  if costmap[index] < LETHAL_COST:
    return True
  
  return False

def sample_points(node_list, costmap, start_index, goal_index, width):
  index_points = random.sample(node_list, N_SAMPLE)
  sample_x = []; sample_y = []

  sample_x.append(start_index % width)
  sample_x.append(goal_index % width)
  sample_y.append(start_index // width)
  sample_y.append(goal_index // width)

  for i, point in enumerate(index_points):
      if not check_collision(point, costmap):
          point = random.choice(node_list)
          while(point in index_points):
              point = random.choice(node_list)

      x = point % width
      y = point // width
      sample_x.append(x); sample_y.append(y)

  return sample_x, sample_y

def generate_graph(sample_x, sample_y, costmap, width):
  """ Function to generate graph from randomly sampled points """
  graph = dict()
  kd_tree = KDTree(np.vstack((sample_x, sample_y)).T)

  for (i, ix, iy) in zip(range(N_SAMPLE + 2), sample_x, sample_y):
      dists, indices = kd_tree.query([ix, iy], k = N_SAMPLE - 3)
      iindex = iy * width + ix
      edge_id = []

      for j in range(len(indices)):
          nx = sample_x[indices[j]]
          ny = sample_y[indices[j]]
          edge_id.append(indices[j])
      
      graph[iindex] = edge_id

  return graph

def euclidean_distance(index, goal_index, width):
  """ Function to calculate Euclidean Distance """
  index_x = index % width
  index_y = int(index // width)
  goal_x = goal_index % width
  goal_y = int(goal_index // width)

  distance = (index_x - goal_x) ** 2 + (index_y - goal_y) ** 2
  return math.sqrt(distance)

def get_distance_and_angle(index, goal_index, width):
  """ Function to calculate distance and angle """
  index_x = index % width
  index_y = int(index // width)
  goal_x = goal_index % width
  goal_y = int(goal_index // width)

  dx = goal_x - index_x
  dy = goal_y - index_y

  distance = dx ** 2 + dy ** 2
  angle = math.atan2(dy, dx)
  return math.sqrt(distance), angle

def dijkstra_algorithm(start_index, goal_index, sample_x, sample_y, 
                       graph, costmap, width):
  """
  Dijkstra Algorithm
  """
  open_list = []
  closed_list = set()
  parents = dict()

  g_costs = dict()
  g_costs[start_index] = 0

  open_list.append([start_index, 0])

  shortest_path = []

  path_found = False
  # Main loop, executes as long as there are still nodes inside open_list
  while open_list:
      open_list.sort(key = lambda x: x[1]) 
      current_index = open_list.pop(0)[0]
      closed_list.add(current_index)

      if current_index == goal_index:
          path_found = True
          break

      # Loop neighbors
      for nid in graph[current_index]:
          x = sample_x[nid]; y = sample_y[nid]
          neighbor_index = width * y + x 
          step_cost = euclidean_distance(current_index, neighbor_index, width)

          if neighbor_index in closed_list:
            continue

          g_cost = g_costs[current_index] + step_cost

          in_open_list = False
          for idx, element in enumerate(open_list):
            if element[0] == neighbor_index:
              in_open_list = True
              break

          if in_open_list:
            if g_cost < g_costs[neighbor_index]:
              g_costs[neighbor_index] = g_cost
              parents[neighbor_index] = current_index
              open_list[idx] = [neighbor_index, g_cost]
          else:
            g_costs[neighbor_index] = g_cost
            parents[neighbor_index] = current_index
            open_list.append([neighbor_index, g_cost])

  if not path_found:
      return shortest_path

  # Reconstruct path by working backwards from target
  if path_found:
      node = goal_index
      shortest_path.append(goal_index)
      while node != start_index:
          shortest_path.append(node)
          node = parents[node]

  # reverse list
  shortest_path.append(start_index)
  shortest_path = shortest_path[::-1]
  return shortest_path

def interpolate(from_index, to_index, width, extend_length = float('inf')):
  """ 
  Function to interpolate a path between nearest node 
  and sampled node 
  """
  path_index = []

  distance, angle = get_distance_and_angle(from_index, to_index, width)

  if extend_length > distance:
    extend_length = distance

  n_expand = extend_length // PATH_RESOLUTION

  x = from_index % width
  y = from_index // width
  index = from_index

  for _ in range(int(n_expand)):
    x += PATH_RESOLUTION * math.cos(angle)
    y += PATH_RESOLUTION * math.sin(angle)
    next_index = int(y) * width + int(x)
    if index != next_index:
      index = next_index
      path_index.append(index)

  if distance == extend_length:
    index = to_index
    path_index.append(to_index)

  return path_index


def prm(start_index, goal_index, width, height, costmap, 
        resolution, origin, grid_viz):
  """
  Performs Probabilistic Road Map Planning to search for a path
  """

  index_list = init_nodes(start_index, goal_index, width)

  # Sample Points
  sample_x, sample_y = sample_points(index_list, costmap, start_index, goal_index, width)
  for x, y in zip(sample_x, sample_y):
    i = y * width + x
    grid_viz.set_color(i, "pale yellow")

  # Generate graph
  graph = generate_graph(sample_x, sample_y, costmap, width)

  # Find Shortest Path
  path = dijkstra_algorithm(start_index, goal_index, sample_x, sample_y, graph, costmap, width)
  for index in path:
    grid_viz.set_color(index, "lime_green")

  final_path = []
  for i in range(len(path) - 1):
    index_path = interpolate(path[i], path[i + 1], width)
    for index in index_path:
      final_path.append(index)
      grid_viz.set_color(index, "orange")

  return final_path
