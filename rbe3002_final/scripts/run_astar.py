#!/usr/bin/python
import math
import rospy
import inspect
import time
import tf
import sys
from nav_msgs.msg import Path, OccupancyGrid
from nav_msgs.srv import GetPlan
from geometry_msgs.msg import Twist, Pose, PoseStamped, PoseWithCovarianceStamped
from move_base_msgs.msg import MoveBaseActionGoal
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

def get_goal_set(goal_msg):
  global goal_list
  global goal_blacklist
  global pose
  while True:
    try:
      if pose: break
    except:
      pass
  goals = goal_msg.poses
  goal_list = []
  for goal in goals:
    good = True
    for bad in goal_blacklist:
      goalx = goal.pose.position.x
      goaly = goal.pose.position.y
      badx = bad.pose.position.x
      bady = bad.pose.position.y
      if ((badx - goalx) ** 2 + (bady - goaly) ** 2) < 0.1:
        print "Bad goal :("
        good = False
        break
    print "Hello, looping through candidates."
    distance = pose_dist(pose, goal)
    if good:
      print "Goad goal :)"
      goal_list.append((distance, goal))
  # This sort works because the individual tuples, when compared, will first
  # compare by distance and only compare the goal objects as a tie-breaker.
  if len(goal_list) == 0:
    sys.exit()
    #goal_blacklist = []
  goal_list.sort()
  print "Got new set of goals."

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
  global goal_list, goal_blacklist
  goal = None
  rospy.init_node('run_astar')
  tf_list = tf.TransformListener()
  service_name = rospy.get_param("~service_name", "astar")
  rospy.wait_for_service(service_name)
  do_astar = rospy.ServiceProxy(service_name, GetPlan)
  goal_name = rospy.get_param("~goal_name", "/astar_goal")
  rospy.Subscriber(goal_name, PoseStamped, get_goal, queue_size=10)
  goal_list = None
  goal_blacklist = []
  if rospy.has_param("~goal_set"):
    goal_list_topic = rospy.get_param("~goal_set", "/frontier_path")
    rospy.Subscriber(goal_list_topic, Path, get_goal_set, queue_size=10)
  path_pub_name = rospy.get_param("~path_pub", "/astar_continuous_path")
  path_pub = rospy.Publisher(path_pub_name, Path, queue_size=10)
  go_pub_name = rospy.get_param("~go_pub", "/waypoint_global")
  go_pub = rospy.Publisher(go_pub_name, PoseStamped, queue_size=10)
  #go_pub = rospy.Publisher(go_pub_name, MoveBaseActionGoal, queue_size=10)
  stop_pub = rospy.Publisher("/done_navigate", Empty, queue_size=10)

  rospy.sleep(rospy.Duration(1, 0))
  # Periodically update the pose information.
  rospy.Timer(rospy.Duration(0.02), updatePose)

  rospy.sleep(rospy.Duration(1, 0))

  # Loop and constantly call service.
  while not rospy.is_shutdown():
    rospy.sleep(5.0)
    path = None

    # Get the desired goal, if we are using a goal_set.
    try:
      while path == None:
        full_goal = ()
        if goal_list:
          new_goal = True
          for option in goal_list:
            if goal and pose_dist(option[1], goal) < 0.1:
              new_goal = False
          if new_goal:
            for option in goal_list:
              if pose_dist(option[1], pose) > 0.3:
                full_goal = option
                goal = option[1]
                new_goal = False
                break
            if new_goal:
              full_goal = goal_list[0]
              goal = goal_list[0][1]
        if len(goal_list) == 0:
          print "DDDDDOOOOOOOOOOOOOOOOOONNNNNNNNNEEEEEEEEEEEEE"
          sys.exit()
        dist = pose_dist(pose, goal)
        print dist
        if dist > 0.3 or goal_name != "/astar_goal":
          path = do_astar(pose, goal, 0.0).plan
          # If we failed to compute a path and have a goal_list, try again.
          if not path.poses and goal_list:
            print "Rejecting goal."
            goal_blacklist.append(goal)
            goal_list.remove(full_goal)
            path = None
          elif not path.poses:
            break
        else:
          stop_pub.publish(Empty())
          break
    except Exception as exc:
      print goal_name, exc
    if False and path:
      path_pub.publish(path)
      go_pub.publish(path.poses[0])
      # Iterate through path and find the two closest nodes to pose that pose
      # is between and then return the second of them.
      if len(path.poses) > 2:
        # go_pub.publish(path.poses[-1])
        published = False
        for point in reversed(path.poses):
          if pose_dist(point, pose) > 1.5:
            break
          elif pose_dist(point, pose) < 0.3:
            continue
          if between(path.poses[-1], pose, point):
            orient = path.poses[-1].pose.orientation
            quat = [orient.x, orient.y, orient.z, orient.w]
            print tf.transformations.euler_from_quaternion(quat)
            published = True
            go_pub.publish(point)
            break
        if not published:
          for point in reversed(path.poses[:-1]):
            dist = pose_dist(point, pose)
            if dist > 0.7:
              orient = path.poses[-1].pose.orientation
              quat = [orient.x, orient.y, orient.z, orient.w]
              print tf.transformations.euler_from_quaternion(quat)
              go_pub.publish(point)
              published = True
              break
          if not published: go_pub.publish(path.poses[0])
      elif len(path.poses) == 2: go_pub.publish(path.poses[0])
      elif len(path.poses) == 1:
        stop_pub.publish(Empty())
    else:
      go_pub.publish(goal)
