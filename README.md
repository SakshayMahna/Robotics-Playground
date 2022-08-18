# Robotics-Playground
Playground to benchmark algorithms on Robot Simulations.

The repository contains code used in the video series: Motion Planning for Robots. Click [here](https://youtube.com/playlist?list=PL0sla3wvhSnYNAyp0-OQmTMyO2POZRSe-) to go the playlist.

# Contents

Instructions to run the simulation are in the respective workspace directory

| Workspace      | Video Title | Video Link | 
| ----------- | ----------- | ----------- |
| `turtlebot3_ws`      | A Star | [Link](https://youtu.be/nbaSzCnmyec) |
| `ur5_ws`   | Genetic Algorithm | [Link](https://youtu.be/RMHcwaTtvLg) |
| `spot_ws`  | RRT | [Link](https://youtu.be/_KD_2IsWslM) |
| `driving_ws` | Frenet Frames | [Link](https://youtu.be/DhP3jiC9YX0) |

Initialize git submodules:

    cd Robotics-Playground
    git submodule update --init --recursive
    
Install ROS dependency from a certain ws:

    cd <certain_ws>
    rosdep install --from-paths src --ignore-src -r -y

Change permission for python scripts:

    cd Robotics-Playground
    find . -name *.py -exec chmod +x {} \;
    
To let git ignore permission changes within the repo:

    cd Robotics-Playground
    git config core.fileMode false
