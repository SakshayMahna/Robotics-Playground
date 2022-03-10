# Self Driving Car Workspace

This workspace has been tested with ROS Melodic. And may work for ROS Kinetic. This directory contains all the code used for Frenet Frame based Path Planning.

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

3. Start the Car simulation, and keep the terminal running. The Gazebo window showing the Robot Car in an empty environment would open. Zoom out in the environment to see the robot car. In addition to that, the Rviz visualization window would also open, showing the robot car.

```bash
roslaunch car_demo demo.launch
```

4. Keep the previous terminal running, and open a new terminal. Source the files, as given in 2. Then, start the Path Planning Node, and keep the terminal running. The code would take some time to run. As it runs, the background color of the RViz window would change from black to white. The Path to follow would appear, and eventually the robot will start following that path.

```bash
roslaunch frenet_planner frenet_planner.launch
```