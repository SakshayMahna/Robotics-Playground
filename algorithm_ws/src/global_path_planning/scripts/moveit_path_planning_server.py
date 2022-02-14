#!/usr/bin/env python

import rospy
from pp_msgs.srv import MoveItPlugin, MoveItPluginResponse
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from geometry_msgs.msg import Twist
from gridviz import GridViz
from algorithms.ga import ga, straight

def make_plan(req):
  ''' 
  Callback function used by the service server to process
  requests from clients. It returns a msg of type PathPlanningPluginResponse
  ''' 
  # Get Request Parameters
  joint_names = req.joint_names
  start_joint_values = req.start_joint
  goal_joint_values = req.goal_joint

  # time statistics
  start_time = rospy.Time.now()

  # Solve
  rospy.loginfo('++++++++ Started Path Planning ++++++++')
  joint_values = ga(start_joint_values, goal_joint_values)

  execution_time = rospy.Time.now() - start_time
  print("\n")
  rospy.loginfo('++++++++ Path Planning execution metrics ++++++++')
  rospy.loginfo('Total execution time: %s seconds', str(execution_time.to_sec()))
  rospy.loginfo('++++++++++++++++++++++++++++++++++++++++++++')
  print("\n")
  rospy.loginfo('Path sent!')

  resp = MoveItPluginResponse()
  resp.plan = JointTrajectory()
  resp.plan.joint_names = joint_names
  resp.plan.header.stamp = rospy.Time.now()
  for position in joint_values:
    trajectory_point = JointTrajectoryPoint()
    trajectory_point.positions = position
    resp.plan.points.append(trajectory_point)

  return resp

def clean_shutdown():
  cmd_vel.publish(Twist())
  rospy.sleep(1)

if __name__ == '__main__':
  rospy.init_node('path_planning_service_server', log_level=rospy.INFO, anonymous=False)
  make_plan_service = rospy.Service("/move_group/make_plan", MoveItPlugin, make_plan)
  cmd_vel = rospy.Publisher('/cmd_vel', Twist, queue_size=5)
  rospy.on_shutdown(clean_shutdown)

  while not rospy.core.is_shutdown():
    rospy.rostime.wallsleep(0.5)
  rospy.Timer(rospy.Duration(2), rospy.signal_shutdown('Shutting down'), oneshot=True)
