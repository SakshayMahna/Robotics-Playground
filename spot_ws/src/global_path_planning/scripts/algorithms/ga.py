#! /usr/bin/env python

from matplotlib import pyplot as plt
from mpl_toolkits.mplot3d import axes3d, Axes3D
import sys
import rospy
import moveit_commander
from urdf_parser_py.urdf import URDF
from pykdl_utils.kdl_kinematics import KDLKinematics
from hrl_geom import transformations
import pygad
from math import sqrt
from copy import deepcopy
import bezier
import numpy as np

# Initialize Kinematics
robot_urdf = URDF.from_xml_string(rospy.get_param("robot_description"))
kdl_kin = KDLKinematics(robot_urdf, "world", "tool0")

def euclidean_distance(point1, point2):
    dist = (point1[0] - point2[0]) ** 2 + (point1[1] - point2[1]) ** 2 + (point1[2] - point2[2]) ** 2
    return dist

def calculate_fitness(start_transform, goal_transform, solution):
    # Iterate over points
    points = []
    points.append([start_transform[0, 3], start_transform[1, 3], start_transform[2, 3]] + \
                  list(transformations.euler_from_matrix(start_transform)))
    for i in range(int(len(solution) / 6) - 1):
        transform = kdl_kin.forward(solution[i*6 : (i + 1)*6])
        points.append([transform[0, 3], transform[1, 3], transform[2, 3]] + \
                  list(transformations.euler_from_matrix(start_transform)))
    points.append([goal_transform[0, 3], goal_transform[1, 3], goal_transform[2, 3]] + \
                  list(transformations.euler_from_matrix(start_transform)))

    # Calculate Curve Length
    curve = bezier.Curve.from_nodes(np.transpose(points))

    return 1 / curve.length

def ga(start_joint_values, goal_joint_values):
    start_transform = kdl_kin.forward(start_joint_values)
    goal_transform = kdl_kin.forward(goal_joint_values)

    def fitness_func(solution, solution_idx):
        return calculate_fitness(start_transform, goal_transform, solution)

    fitness_function = fitness_func

    num_generations = 50
    population_size = 100

    num_genes = 3 * 6
    num_parents_mating = 2

    init_range_low = -3.14
    init_range_high = 3.14

    parent_selection_type = "rws"

    crossover_type = "single_point"
    crossover_probability = 0.9

    mutation_type = "random"
    mutation_probability = 0.1

    ga_instance = pygad.GA(num_generations=num_generations,
                        sol_per_pop = population_size,
                        fitness_func=fitness_function,
                        num_genes=num_genes,
                        init_range_low=init_range_low,
                        init_range_high=init_range_high,
                        parent_selection_type=parent_selection_type,
                        num_parents_mating = num_parents_mating,
                        crossover_type=crossover_type,
                        crossover_probability = crossover_probability,
                        mutation_type=mutation_type,
                        mutation_probability = mutation_probability)
    
    ga_instance.run()
    solution, solution_fitness, solution_idx = ga_instance.best_solution()

    points = []
    points.append([start_transform[0, 3], start_transform[1, 3], start_transform[2, 3]] + \
                  list(transformations.euler_from_matrix(start_transform)))
    for i in range(int(len(solution) / 6) - 1):
        transform = kdl_kin.forward(solution[i*6 : (i + 1) * 6])
        points.append([transform[0, 3], transform[1, 3], transform[2, 3]] + \
                  list(transformations.euler_from_matrix(start_transform)))
    points.append([goal_transform[0, 3], goal_transform[1, 3], goal_transform[2, 3]] + \
                  list(transformations.euler_from_matrix(start_transform)))
        
    curve = bezier.Curve.from_nodes(np.transpose(points))
    s_vals = np.linspace(0.0, 1.0, 100)
    curve_points = np.transpose(curve.evaluate_multi(s_vals))

    joint_values = []
    joint_values.append(start_joint_values)
    for point in curve_points:
        transform = transformations.compose_matrix(angles = point[:3], translate = point[3:])
        joint_value = kdl_kin.inverse(transform, joint_values[-1])
        if joint_value is not None:
            joint_values.append(joint_value)
    joint_values.append(goal_joint_values)

    return joint_values

def straight(start_joint_values, goal_joint_values):
    joint_values = []
    for i in range(31):
        joint_value = []
        for j in range(6):
            value = start_joint_values[j] + i * (goal_joint_values[j] - start_joint_values[j]) / 30
            joint_value.append(value)
        joint_values.append(joint_value)

    return joint_values