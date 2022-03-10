# UR5 Workspace

This workspace has been tested for ROS Noetic. And may work for ROS Melodic and ROS Kinetic. This directory contains all the code used for Genetic Algorithm based Path Planning.

## Dependencies
Install the ROS MoveIt

```bash
sudo apt-get install ros-<dist>-moveit
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

3. Start the UR5 simulation, and keep the terminal running. The RViz visualization window would show up with the Robot on right side of the window.

```bash
roslaunch ur5_moveit_config demo.launch
```

4. Keep the previous terminal running, and open a new terminal. Source the files, as given in 2. Then, start the Path Planning Server for MoveIt. There are two things to check for proper initialization. First, in the RViz window, a controller bubble would appear on the end effector of the robot. It can be moved to adjust the goal position. Second, in the previous terminal, the MoveGroup would initialize, giving the log `You can start planning now!`.

```bash
rosrun global_path_planning moveit_path_planning_server.py
```

5. In the RViz window, drag the controller bubble to set a goal position.

6. On the left side of RViz window, click the Plan button. This would start the algorithm to plan a path. To plan the executed path, click Execute button. To perform these operations one after the other automatically, click the Plan & Execute Button.