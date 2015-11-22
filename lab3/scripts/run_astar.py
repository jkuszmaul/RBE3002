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

def pose_dist(start, end):
  """
    X-Y distance between two poses.
  """
  dist = math.sqrt((start.pose.position.y - end.pose.position.y) ** 2 +
                   (start.pose.position.x - end.pose.position.x) ** 2)
  return dist

def between(start, cur, end):
  """
    Whether cur is "between" start and end.
  """
  curpos = cur.pose.position
  startpos = start.pose.position
  endpos = end.pose.position
  xbet = ((curpos.x < endpos.x and curpos.x > startpos.x) or
          (curpos.x > endpos.x and curpos.x < startpos.x))
  ybet = ((curpos.y < endpos.y and curpos.y > startpos.y) or
          (curpos.y > endpos.y and curpos.y < startpos.y))
  return xbet or ybet

if __name__ == '__main__':
  global tf_list
  global goal
  global pose
  rospy.init_node('run_astar')
  tf_list = tf.TransformListener()
  service_name = rospy.get_param("~service_name", "astar")
  rospy.wait_for_service(service_name)
  do_astar = rospy.ServiceProxy(service_name, GetPlan)
  goal_name = rospy.get_param("~goal_name", "/astar_goal")
  rospy.Subscriber(goal_name, PoseStamped, get_goal, queue_size=10)
  path_pub_name = rospy.get_param("~path_pub", "/astar_continuous_path")
  path_pub = rospy.Publisher(path_pub_name, Path, queue_size=10)
  go_pub_name = rospy.get_param("~go_pub", "/waypoint_global")
  go_pub = rospy.Publisher(go_pub_name, PoseStamped, queue_size=10)
  stop_pub = rospy.Publisher("/done_navigate", Empty, queue_size=10)

  rospy.sleep(rospy.Duration(1, 0))
  # Periodically update the pose information.
  rospy.Timer(rospy.Duration(0.02), updatePose)

  rospy.sleep(rospy.Duration(1, 0))

  # Loop and constantly call service.
  while not rospy.is_shutdown():
    rospy.sleep(rospy.Duration(1, 0))
    path = None
    try:
      dist = pose_dist(pose, goal)
      print dist
      if dist > 0.3 or goal_name != "/astar_goal":
        path = do_astar(pose, goal, 0.0).plan
      else:
        stop_pub.publish(Empty())
    except Exception as exc:
      print exc
    if path:
      path_pub.publish(path)
      # Iterate through path and find the two closest nodes to pose that pose
      # is between and then return the second of them.
      if len(path.poses) > 2:
        published = False
        for point in reversed(path.poses):
          if pose_dist(point, pose) > 1.5:
            break
          elif pose_dist(point, pose) < 0.3:
            continue
          if between(path.poses[-1], pose, point):
            published = True
            go_pub.publish(point)
            break
        if not published:
          for point in reversed(path.poses[:-1]):
            dist = pose_dist(point, pose)
            print " ", dist
            if dist > 0.7:
              go_pub.publish(point)
              published = True
              print "PUBLISHING~~~~"
              break
          if not published: go_pub.publish(path.poses[0])
      elif len(path.poses) == 2: go_pub.publish(path.poses[0])
      elif len(path.poses) == 1:
        stop_pub.publish(Empty())
