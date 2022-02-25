#! /usr/bin/env python

import rospy
import random
from math import sqrt, atan2, floor, cos, sin
from scipy.spatial import KDTree

max_iter = 10000
goal_sample_rate = 30
lethal_cost = 150
path_resolution = 1

def index_to_world(index, resolution, origin, width):
  """ Convert index to world coordinates """
  x = int(index % width); y = int(index // width)
  x = resolution * x + origin[0] + resolution / 2
  y = resolution * y + origin[1] + resolution / 2
  return x, y

def world_to_index(x, y, origin, resolution, width):
  """ Convert World coordinates to index """
  x = (x - origin[0]) / resolution
  y = (y - origin[1]) / resolution
  return y * width + x

def euclidean_distance(index, goal_index, width):
  """ Function to calculate Euclidean Distance """
  index_x = index % width
  index_y = int(index // width)
  goal_x = goal_index % width
  goal_y = int(goal_index // width)

  distance = (index_x - goal_x) ** 2 + (index_y - goal_y) ** 2
  return sqrt(distance)

def get_distance_and_angle(index, goal_index, width):
  """ Function to calculate distance and angle """
  index_x = index % width
  index_y = int(index // width)
  goal_x = goal_index % width
  goal_y = int(goal_index // width)

  dx = goal_x - index_x
  dy = goal_y - index_y

  distance = dx ** 2 + dy ** 2
  angle = atan2(dy, dx)
  return sqrt(distance), angle

def check_collision(index, costmap):
  """ Check Collision, if safe return True """
  if costmap[index] < lethal_cost:
    return True
  
  return False

def check_inside_area(index, index_list):
  """ Check Inside, return True if inside """
  if index in index_list:
    return True
  
  return False

def get_nearest_node_index(node_list, node, width):
  """ Function to get the index of the nearest node """
  distance = [euclidean_distance(node_, node, width) for node_ in node_list]
  min_index = distance.index(min(distance))
  return min_index

def get_random_node(index_list, start_index, goal_index):
  """ Function to get a random node """
  if random.randint(0, 100) > goal_sample_rate:
    node = random.choice(index_list)
  else:
    node = goal_index

  return node

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

def interpolate(from_index, to_index, width, extend_length = float('inf')):
  """ 
  Function to interpolate a path between nearest node 
  and sampled node 
  """
  path_index = []

  distance, angle = get_distance_and_angle(from_index, to_index, width)

  if extend_length > distance:
    extend_length = distance

  n_expand = extend_length // path_resolution

  x = from_index % width
  y = from_index // width
  index = from_index

  for _ in range(int(n_expand)):
    x += path_resolution * cos(angle)
    y += path_resolution * sin(angle)
    next_index = int(y) * width + int(x)
    if index != next_index:
      index = next_index
      path_index.append(index)

  if distance == extend_length:
    index = to_index
    path_index.append(to_index)

  return index, path_index

def rrt(start_index, goal_index, width, height, costmap, 
        resolution, origin, grid_viz):
  """
  Performs RRT Path Planning to search for a path
  """

  # Node List
  node_list = [start_index]
  index_list = init_nodes(start_index, goal_index, width)
  parents = dict()
  path_found = False

  # Expand Distance
  expand_distance = 15
  
  for i in range(max_iter):
    next_node = get_random_node(index_list, start_index, goal_index)
    nearest_index = get_nearest_node_index(node_list, next_node, width)
    nearest_node = node_list[nearest_index]

    new_node, path = interpolate(nearest_node, next_node, width, expand_distance)

    if new_node in node_list:
      continue

    if not check_inside_area(new_node, index_list):
      continue
    if not check_collision(new_node, costmap):
      continue

    grid_viz.set_color(new_node, "pale yellow")

    prev_index = path[0]
    grid_viz.set_color(path[0], "orange")
    node_list.append(path[0])
    parents[prev_index] = start_index
    for path_index in path[1:]:
      node_list.append(path_index)
      if prev_index != path_index:
        parents[path_index] = prev_index
      prev_index = path_index
      grid_viz.set_color(path_index, "orange")

    if new_node == goal_index:
      path_found = True
      break
  
  # Reconstruct path by working backwards from target
  shortest_path = []
  if path_found:
      node = goal_index
      while node != start_index:
          shortest_path.append(node)
          # get next node
          node = parents[node]
          
  # reverse list
  shortest_path.append(start_index)
  shortest_path = shortest_path[::-1]
  return shortest_path      