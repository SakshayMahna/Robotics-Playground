# Robotics-Playground
Playground to benchmark algorithms on Robot Simulations.

## Usage

Initialize git submodules:

    cd Robotics-Playground
    git submodule update --init --recursive
    
Change permission for python scripts:

    find . -name *.py -exec chmod +x {} \;
        
Let git ignore permission changes within the repo:

    git config core.filemode false
    git submodule foreach --recursive git config core.filemode false
   
Install ROS dependency in a certain ws:

    cd <certain_ws>
    rosdep install --from-paths src --ignore-src -r -y

## Tutorials

The repository contains code used in the video series: Motion Planning for Robots. Click [here](https://youtube.com/playlist?list=PL0sla3wvhSnYNAyp0-OQmTMyO2POZRSe-) to go the playlist.

Instructions to run the simulation are in the respective workspace directory

| Workspace      | Video Title | Video Link | 
| ----------- | ----------- | ----------- |
| `turtlebot3_ws`      | A Star | [Link](https://youtu.be/nbaSzCnmyec) |
| `ur5_ws`   | Genetic Algorithm | [Link](https://youtu.be/RMHcwaTtvLg) |
| `spot_ws`  | RRT | [Link](https://youtu.be/_KD_2IsWslM) |
| `driving_ws` | Frenet Frames | [Link](https://youtu.be/DhP3jiC9YX0) |

