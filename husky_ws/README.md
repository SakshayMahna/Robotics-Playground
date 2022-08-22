# Husky Workspace

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

3. Start the Husky simulation, and keep the terminal running. The Gazebo window showing the Husky Robot in an environment would open.

```bash
TODO
roslaunch f1_gazebo simple_circuit.launch
```

4. Keep the previous terminal running, and open a new terminal. Source the files, as given in 2. Then, start the Path Planning Node, and keep the terminal running.

```bash
TODO
rosrun local_navigation local_navigation.py 
```
