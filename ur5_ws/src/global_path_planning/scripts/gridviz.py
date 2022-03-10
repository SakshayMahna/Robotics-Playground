#!/usr/bin/env python

"""
Rviz PointCloud2 visualization marker for path planners in a grid map
Author: Roberto Zegers R.
Copyright: Copyright (c) 2021, Roberto Zegers R.
License: BSD-3-Clause
Date: March 2021
"""

import rospy
from sensor_msgs import point_cloud2
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import Header

class GridViz:
    def __init__(self, flat_map, resolution, origin, start_idx, goal_idx, width, id=0, frame="map"):
        self.flat_map = flat_map
        self.map_resolution = resolution
        self.map_origin = origin
        self.map_width = width
        self.start = start_idx
        self.goal = goal_idx
        self.id = id
        self.rgba_colors = {'green' : 4278255360, 'red' : 4294901760, 'blue' : 4278190335,
                            'pale yellow' : 4293918464, 'lime_green' : 4284802916, 'orange' : 4294944000}
        self.plot_cloud = rospy.Publisher('/grid_viz', PointCloud2, queue_size=100)
        self.header = Header()
        self.header.frame_id = frame
        self.field_x = PointField('x', 0, PointField.FLOAT32, 1)
        self.field_y = PointField('y', 4, PointField.FLOAT32, 1)
        self.field_z = PointField('z', 8, PointField.FLOAT32, 1)
        self.field_rgba = PointField('rgba', 12, PointField.UINT32, 1)
        self.fields = [self.field_x, self.field_y, self.field_z, self.field_rgba]
        self.points = []
        self.init_points()

    def init_points(self):
        for idx, value in enumerate(self.flat_map):
            xyz_point = self.indexToWorld(idx)
            self.points.append(xyz_point + [0])
        self.points[self.start][2] = 0.1
        self.points[self.start][3] = self.rgba_colors['blue']
        self.points[self.goal][2] = 0.1
        self.points[self.goal][3] = self.rgba_colors['red']

    def set_color(self, idx, color):
        if idx == self.start or idx == self.goal:
          pass
        else:
          self.points[idx][3] = self.rgba_colors[color]
        pc2 = point_cloud2.create_cloud(self.header, self.fields, self.points)
        pc2.header.stamp = rospy.Time.now()
        self.plot_cloud.publish(pc2)

    def indexToWorld(self, array_index):
        grid_cell_x = array_index % self.map_width
        grid_cell_y = array_index // self.map_width
        x = self.map_resolution * grid_cell_x + self.map_origin[0] + self.map_resolution / 2
        y = self.map_resolution * grid_cell_y + self.map_origin[1] + self.map_resolution / 2
        z = 0.0
        return [x,y,z]
