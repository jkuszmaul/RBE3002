#!/usr/bin/python
import math
import rospy
import inspect
import time
import tf
import actionlib
import sys
from nav_msgs.msg import Path, OccupancyGrid
from nav_msgs.srv import GetPlan
from kobuki_msgs.msg import Sound
from geometry_msgs.msg import Twist, Pose, PoseStamped, PoseWithCovarianceStamped
from move_base_msgs.msg import MoveBaseAction, MoveBaseActionGoal, MoveBaseGoal
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

all_bad_cnt = 0
def get_goal_set(goal_msg):
  global goal_list
  global goal_blacklist
  global pose
  global all_bad_cnt
  global sounds
  global goal_lock
  while goal_lock: continue
  goal_lock = True
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
    goal_blacklist = []
  goal_list.sort()
  print "Got new set of goals."
  goal_lock = False

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

def drivePub(linear, angular):
  global vel_pub
  msg = Twist()
  msg.angular.z = angular
  msg.linear.x = linear
  vel_pub.publish(msg)

if __name__ == '__main__':
  global tf_list
  global goal
  global goal_lock
  global pose
  global goal_list, goal_blacklist
  global vel_pub
  global sounds
  goal = None
  goal_lock = False
  rospy.init_node('run_astar')
  tf_list = tf.TransformListener()
  goal_list = None
  goal_blacklist = []
  goal_list_topic = rospy.get_param("~goal_set", "/frontier_path")
  rospy.Subscriber(goal_list_topic, Path, get_goal_set, queue_size=10)
  do_astar = rospy.ServiceProxy("astar", GetPlan)
  sounds = rospy.Publisher("/mobile_base/commands/sound", Sound, queue_size=10)
  go_pub = rospy.Publisher("/goal_for_display", PoseStamped, queue_size=10)
  vel_pub = rospy.Publisher('/cmd_vel_mux/input/teleop', Twist, queue_size=100)
  go_action = actionlib.SimpleActionClient('move_base', MoveBaseAction)
  go_action.wait_for_server()
  #test_goal = MoveBaseGoal()
  #test_goal.target_pose.header.frame_id = 'map'
  #test_goal.target_pose.pose.position.x = 100000
  #test_goal.target_pose.pose.orientation.w = 1
  #go_action.send_goal(test_goal)
  #print go_action.wait_for_result()
  #rospy.sleep(10000000000000000000000)

  rospy.sleep(rospy.Duration(1, 0))
  # Periodically update the pose information.
  rospy.Timer(rospy.Duration(0.02), updatePose)
  print "HELLO!!!!!!!!!!!!!!!!"
  # Do full circle spin.
  for _ in xrange(400):
    drivePub(0, 0.5)
    rospy.sleep(.05)
    if rospy.is_shutdown():
      break

  rospy.sleep(rospy.Duration(1, 0))

  do_circle = True
  all_bad_cnt = 0
  # Loop and constantly call service.
  while not rospy.is_shutdown():
    # Do full circle spin.
    if do_circle:
      for _ in xrange(150):
        drivePub(0, 0.5)
        rospy.sleep(.05)
        if rospy.is_shutdown():
          break

    # Get the desired goal, if we are using a goal_set.
    try:
      full_goal = ()
      if goal_list:
        while goal_lock: continue
        goal_lock = True
        #new_goal = True
        #for option in goal_list:
        #  if goal and pose_dist(option[1], goal) < 0.1:
        #    new_goal = False
        #if new_goal:
        #  for option in goal_list:
        #    if pose_dist(option[1], pose) > 0.3:
        #      full_goal = option
        #      new_goal = False
        #      break
        #  if new_goal:
        #    full_goal = goal_list[0]
        test_plan = []
        while not len(test_plan):
          if len(goal_list) == 0:
            all_bad_cnt += 1
            if all_bad_cnt > 2:
              sounds.publish(Sound(0))
              rospy.sleep(2.)
              sounds.publish(Sound(1))
              rospy.sleep(2.)
              sys.exit()
            break
          goal = goal_list.pop(0)[1]
          test_plan = do_astar(goal, pose, 0.0).plan.poses
          print test_plan
        goal_lock = False
      dist = pose_dist(pose, goal)
      print dist
      #path = do_astar(pose, goal, 0.0).plan
      go_pub.publish(goal)
      pub_goal = MoveBaseGoal()
      pub_goal.target_pose = goal
      go_action.send_goal(pub_goal)
      result = go_action.wait_for_result()
      goal_blacklist.append(goal)
      goal_list.pop(0)
      if go_action.get_state() == 4:
        do_circle = False
      else:
        do_circle = True
    except Exception as exc:
      print exc

