#!/usr/bin/env python

import sys, os

from vff import VFF
from interfaces.pose3d import ListenerPose3d
from interfaces.laser import ListenerLaser
from interfaces.motors import PublisherMotors

if __name__ == "__main__":
    motors = PublisherMotors("F1ROS/cmd_vel", 4, 0.3)
    pose3d = ListenerPose3d("F1ROS/odom")
    laser = ListenerLaser("F1ROS/laser/scan")

    algorithm = VFF(pose3d, laser, motors)
    algorithm.run()
