#!/usr/bin/python

import rospy, tf
import numpy
from kobuki_msgs.msg import BumperEvent
from geometry_msgs.msg import Twist, PoseStamped
import math
from std_msgs.msg import Empty

class SimplePose(object):
  """
    A 2-dimensional pose object with a position and orientation.
  """
  x = 0
  y = 0
  t = 0
  def __repr__(self):
    return repr(self.x) + ", " + repr(self.y) + ", " + repr(self.t)

def updatePose(event):
  """
    Update the pose from the TF.
  """
  global pose
  global odom_list

  (position, orientation) = odom_list.lookupTransform('map','base_link', rospy.Time(0))
  pose = SimplePose()
  _, _, pose.t = tf.transformations.euler_from_quaternion(orientation)
  pose.x = position[0]
  pose.y = position[1]

  driveUpdate()

def driveUpdate():
  global pose
  global goal
  global stopped
  if stopped:
    drivePub(0, 0.3)
    return
  if goal == None:
    return
#  print "goal: ", goal
#  print "pose: ", pose
  ydiff = goal.y - pose.y
  xdiff = goal.x - pose.x
  goal_angle = math.atan2(ydiff, xdiff)
  angle_error = goal_angle - pose.t
  if angle_error > math.pi: angle_error -= math.pi * 2
  if angle_error < -math.pi: angle_error += math.pi * 2
  goal_dist = math.sqrt(ydiff ** 2 + xdiff ** 2)
#  print "angle error: ", angle_error, "goal dist: ", goal_dist

  # Tuning constants
  angle_threshold = math.pi / 3
  kP_lin = 0.4
  kP_ang = 1

  max_lin = 0.1 if abs(angle_error) > math.pi / 6 else 0.3
  lin_vel = kP_lin * goal_dist + 0.05
  if lin_vel > max_lin: lin_vel = max_lin
  if abs(angle_error) > angle_threshold:
    lin_vel = 0

  ang_vel = kP_ang * angle_error
  ang_vel = numpy.clip(ang_vel, -0.5, 0.5)
  #if goal_dist < 0.1:
  #  ang_vel = 0
  print "Pre-Driving"
  if abs(lin_vel) < 0.25:# and abs(ang_vel) < 0.05:
    drivePub(0, 0.3 * (1 if ang_vel > 0 else -1))
  else:
    print "Driving"
    drivePub(lin_vel, ang_vel)

def goal_callback(event):
  global goal
  global stopped
  stopped = False
  goal = SimplePose()
  orst = event.pose.orientation
  orientation = [orst.x, orst.y, orst.z, orst.w]
  _, _, goal.t = tf.transformations.euler_from_quaternion(orientation)
  goal.x = event.pose.position.x
  goal.y = event.pose.position.y

def stop_robot(event):
  global stopped
  stopped = True

def drivePub(linear, angular):
  global vel_pub
  msg = Twist()
  msg.angular.z = angular
  msg.linear.x = linear
  vel_pub.publish(msg)

# This is the program's main function
if __name__ == '__main__':
  rospy.init_node('drive')

  global odom_list
  global vel_pub

  global stopped
  stopped = False
  goal = None
  # Use this object to get the robot's Odometry
  odom_list = tf.TransformListener()

  rospy.Subscriber('/waypoint', PoseStamped, goal_callback, queue_size=10)
  rospy.Subscriber('/done_navigate', Empty, stop_robot, queue_size=10)
  vel_pub = rospy.Publisher('/cmd_vel_mux/input/teleop', Twist, queue_size=100)

  rospy.sleep(rospy.Duration(1, 0))

  # Do full circle spin.
  for _ in xrange(700):
    drivePub(0, 0.3)
    rospy.sleep(.05)
    if rospy.is_shutdown():
      break
  print "Ready!!"

  #make the robot keep doing something...
  rospy.Timer(rospy.Duration(0.02), updatePose)

  rospy.spin()
