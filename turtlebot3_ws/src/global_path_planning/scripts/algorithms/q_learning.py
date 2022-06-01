#! /usr/bin/env python

import rospy
import numpy as np
from math import sqrt
from algorithms.neighbors import find_neighbors

def create_grid(start_index, goal_index, width):
    start_x = start_index % width
    start_y = int(start_index / width)
    goal_x = goal_index % width
    goal_y = int(goal_index / width)

    x_init = min(start_x, goal_x)
    y_init = min(start_y, goal_y)
    x_end = max(start_x, goal_x)
    y_end = max(start_y, goal_y)

    grid = []
    q_grid = []

    for y in range(x_init, x_end + 1):
        grid.append([])
        q_grid.append([])
        for x in range(y_init, y_end + 1):
            grid[-1].append(1)
            q_grid[-1].append(0)

    grid[x_end - start_x][y_end - start_y] = 2
    grid[x_end - goal_x][y_end - goal_y] = 3

    return np.array(grid)

# Function to get next valid action to take
def get_valid_actions(grid):
    current_pos = np.where(grid == 2)
    x = current_pos[0][0]; y = current_pos[1][0]

    valid_actions = []
    # Up: 0
    if x - 1 >= 0:
        valid_actions.append(0)
    # Down: 2
    if x + 1 < grid.shape[0]:
        valid_actions.append(2)
    # Left: 1
    if y - 1 >= 0:
        valid_actions.append(1)
    # Right: 3
    if y + 1 < grid.shape[1]:
        valid_actions.append(3)

    return valid_actions

# Function to take action and update board
def take_action(action, grid):
    current_pos = np.where(grid == 2)
    x = current_pos[0][0]; y = current_pos[1][0]

    grid[x][y] = 1
    prev_pos = (x, y)

    # Up: 0
    if action == 0:
        grid[x - 1][y] = 2
        current_pos = (x - 1, y)
    # Down: 2
    elif action == 2:
        grid[x + 1][y] = 2
        current_pos = (x + 1, y)
    # Left: 1
    elif action == 1:
        grid[x][y - 1] = 2
        current_pos = (x, y - 1)
    # Right: 3
    elif action == 3:
        grid[x][y + 1] = 2
        current_pos = (x, y + 1)
    else:
        raise Exception("Incorrect action")

    return grid, prev_pos, current_pos

# Function to choose action according to epsilon-greedy policy
def choose_action(grid, q_table, epsilon):
    action = -1
    current_pos = np.where(grid == 2)
    x = current_pos[0][0]; y = current_pos[1][0]

    # Random Action
    if np.random.uniform(0, 1) <= epsilon:
        action = np.random.choice(get_valid_actions(grid))

    # Greedy Action
    else:
        actions = get_valid_actions(grid)
        max_value = -float("inf")
        action = -1

        for a in actions:
            if q_table[a][x][y] > max_value:
                max_value = q_table[a][x][y]
                action = a

    return action

# Function to check if goal is reached
def check_reached(grid, goal_pos):
    current_pos = np.where(grid == 2)
    current_x = current_pos[0][0]; current_y = current_pos[1][0]

    goal_x = goal_pos[0][0]; goal_y = goal_pos[1][0]

    if current_x == goal_x and current_y == goal_y:
        return True

    return False

# Function to print direction table
def print_direction_table(q_table):
    max_table = np.argmax(q_table, axis = 0)
    direction_table = []
    print(max_table)

    # Up, Left, Down, Right
    for i in range(max_table.shape[0]):
        direction_table.append([])
        for j in range(max_table.shape[1]):
            if max_table[i][j] == 0:
                direction_table[-1].append('^')
            elif max_table[i][j] == 1:
                direction_table[-1].append('<')
            elif max_table[i][j] == 2:
                direction_table[-1].append('V')
            elif max_table[i][j] == 3:
                direction_table[-1].append('>')

    print(np.array(direction_table))

# Function to get positions to reach goal
def get_final_positions(start_index, goal_index, width, q_table):
    start_x = start_index % width
    start_y = int(start_index / width)
    goal_x = goal_index % width
    goal_y = int(goal_index / width)

    x_init = min(start_x, goal_x)
    y_init = min(start_y, goal_y)
    x_end = max(start_x, goal_x)
    y_end = max(start_y, goal_y)
    
    grid = create_grid(start_index, goal_index, width)
    goal_pos = np.where(grid == 3)
    indices = []

    while (not check_reached(grid, goal_pos)):
        next_action = choose_action(grid, q_table, 0)
        grid, prev_pos, current_pos = take_action(next_action, grid)
        index = width * (y_end - current_pos[1]) + (x_end - current_pos[0])
        indices.append(index)
        
    return indices

def q_learning(start_index, goal_index, width, height, costmap, resolution, origin, grid_viz, previous_plan_variables):
    ''' 
    Uses Q Learning Algorithm to find a path from start to goal
    '''
    grid = create_grid(start_index, goal_index, width)
    original_grid = np.copy(grid)
    q_table = np.zeros((4, ) + grid.shape)

    episodes = 3000
    total_iterations = 1000
    alpha = 0.8
    gamma = 0.9
    epsilon = 0.7

    print(original_grid)
    for _ in range(episodes):
        grid = np.copy(original_grid)
        goal_pos = np.where(grid == 3)
        iterations = 0

        while (not check_reached(grid, goal_pos)) and (iterations < total_iterations):
            next_action = choose_action(grid, q_table, epsilon)
            grid, prev_pos, current_pos = take_action(next_action, grid)
            reward = 100 if check_reached(grid, goal_pos) else -1

            current_q_value = q_table[next_action][prev_pos[0]][prev_pos[1]]
            max_q_value = np.max(q_table[:, current_pos[0], current_pos[1]])
            q_table[next_action][prev_pos[0]][prev_pos[1]] = (1 - alpha) * current_q_value + \
                                                      alpha * (reward + gamma * max_q_value)

            iterations += 1

    print_direction_table(q_table)
    indices = get_final_positions(start_index, goal_index, width, q_table)
    for index in indices:
        grid_viz.set_color(index,'orange')

    return indices, None