#!/usr/bin/python
import math
import rospy
import inspect
import time
import tf
from nav_msgs.msg import Path, OccupancyGrid
from nav_msgs.srv import GetPlan
from geometry_msgs.msg import Twist, Pose, PoseStamped, PoseWithCovarianceStamped
from std_msgs.msg import Empty

def updatePose(event):
  """
    Periodically updates the robot posed based on the tf information.
  """
  global pose
  global tf_list

  (position, orientation) = tf_list.lookupTransform('map','base_link', rospy.Time(0))
  pose = PoseStamped()
  pose.header.frame_id = 'map'
  pose.pose.position.x = position[0]
  pose.pose.position.y = position[1]
  pose.pose.position.z = position[2]
  pose.pose.orientation.w = 1# = orientation

def get_goal(goal_msg):
  global goal
  goal = goal_msg

if __name__ == '__main__':
  global tf_list
  global goal
  global pose
  rospy.init_node('run_astar')
  tf_list = tf.TransformListener()
  rospy.wait_for_service('astar')
  do_astar = rospy.ServiceProxy('astar', GetPlan)
  rospy.Subscriber('/move_base_simple/goal', PoseStamped, get_goal, queue_size=10)
  path_pub = rospy.Publisher('/astar_continuous_path', Path, queue_size=10)
  go_pub = rospy.Publisher('/waypoint', PoseStamped, queue_size=10)

  rospy.sleep(rospy.Duration(1, 0))
  # Periodically update the pose information.
  rospy.Timer(rospy.Duration(0.02), updatePose)

  rospy.sleep(rospy.Duration(1, 0))

  # Loop and constantly call service.
  while True:
    rospy.sleep(rospy.Duration(1, 0))
    path = None
    try:
      path = do_astar(pose, goal, 0.0).plan
    except Exception as exc:
      print exc
    if path:
      path_pub.publish(path)
      if len(path.poses) >= 2: go_pub.publish(path.poses[-2])
