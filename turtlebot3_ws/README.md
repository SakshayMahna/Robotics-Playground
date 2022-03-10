# TurtleBot3 Workspace

This workspace has been tested with ROS Noetic. And may work for ROS Melodic and ROS Kinetic. This directory contains all the code used for A Star based Path Planning.

## Dependencies
Install the ROS Navigation Stack

```bash
sudo apt-get install ros-<dist>-navigation
```

## Steps to Run

1. Having this directory as the present working directory, build the project

```bash
catkin_make
```

2. Source the files

```bash
source devel/setup.bash
```

3. Start the TurtleBot3 simulation, and keep the terminal running. The Gazebo window showing the TurtleBot3 Robot in an environment would open.

```bash
roslaunch ros_world turtlebot3_world.launch
```

4. Keep the previous terminal running, and open a new terminal. Source the files, as given in 2. Then, start the Path Planning Node for TurtleBot3, and keep the terminal running. The RViz visualization window showing the robot in the same environment in a grid format would show up.

```bash
roslaunch global_path_planning turtlebot3_ros_world.launch
```

5. Keep the previous terminal running, and open a new terminal. Source the files, as given in 2. Then, start the Path Planning Server. To check if the code is running properly. Check the second terminal, in which the Path Planning Node is running. On correct execution, after some time, the log `odom received!` should be visible.

```bash
rosrun global_path_planning path_planning_server.py
```

6. In the RViz window, select the 2D Nav Goal button and select the goal position on the map. The orientation can be set while selecting the goal position and dragging the cursor. On setting the position, the visualization of the selected path planning algorithm would start.

7. By default the astar algorithm would work. To set the path planning algorithm, out of astar, dijkstra or greedy. Go to the file `src/global_path_planning/src/path_planning_server.py`. On line 35, instead of calling the `astar` function, call either `dijkstra` or `greedy` with the same arguments to run the respective algorithm.