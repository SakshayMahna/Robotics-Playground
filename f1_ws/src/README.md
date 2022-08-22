# F1 Workspace

This workspace has been tested with ROS Melodic, ROS Noetic. And may work for ROS Kinetic. This directory contains all the code used for Virtual Force Field path planning.

## Dependencies
To install all the dependencies having this directory as the present working directory, use the command:
```bash
rosdep install --from-paths src --ignore-src -r -y
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

3. Start the F1 simulation, and keep the terminal running. The Gazebo window showing the F1 Robot in an environment would open.

```bash
roslaunch f1_gazebo simple_circuit.launch
```

4. Keep the previous terminal running, and open a new terminal. Source the files, as given in 2. Then, start the Path Planning Node, and keep the terminal running.

```bash
rosrun local_navigation local_navigation.py 
```
