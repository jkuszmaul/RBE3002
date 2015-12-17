#!/usr/bin/python
import math
import numpy
import tf
import rospy
import inspect
import time
from astar import debug
from nav_msgs.msg import Path, OccupancyGrid
from nav_msgs.srv import GetPlan
from geometry_msgs.msg import Twist, Pose, PoseStamped, PoseWithCovarianceStamped
from std_msgs.msg import Empty

"""Frontier detection node
This is a ROS node meant to find all of the frontiers on a map.
It will then publish the pose goal for the frontier nearest the robot.

The basic method of frontier detection is as follows:
  1) Identify all the frontiers. A frontier is a set of adjacent (8-connected)
     grid cells which are in the known map and adjacent (4-connected) to an
     unexplored node. A frontier must contain sufficient nodes to be traversable
     by the robot. A frontier is a list of adjacent nodes.
  2) The "center" of the frontier is the node in the center of the list of
     nodes in the frontier.
  3) Publish the center of the frontier, if there are any. This can then
     be navigated to by whatever your favorite navigation stack is.
"""

class Node(object):
  """
    This is a simple node object; it just has a position and neighbors.
  """
  def __init__(self, x, y, neighbors=None):
    self.x = x
    self.y = y
    self.neighbors = neighbors if neighbors else []

  def string_coord(self):
    return "(" + repr(self.x) + ", " + repr(self.y) + ")"

  def __repr__(self):
    neighbors = "[]"
    if len(self.neighbors):
      neighbors = "[" + self.neighbors[0].string_coord()
      for neighbor in self.neighbors[1:]:
        neighbors += ", " + neighbor.string_coord()
      neighbors += "]"
    return self.string_coord() + ": " + neighbors

def create_hole(rosmap):
  """
    Intentionally creates a hole in the middle of a map for testing purposes.
  """
  empty_size = 10
  width = rosmap.info.width
  height = rosmap.info.height
  middlex = int(width / 2)
  middley = int(height / 2)
  data_list = list(rosmap.data)
  for y in xrange(middley - empty_size, middley + empty_size):
    for x in xrange(middlex - empty_size, middlex + empty_size):
      data_list[y * width + x] = -1
  # Now, get a chunk on the edge.
  for y in xrange(0, empty_size):
    for x in xrange(middlex - empty_size, middlex + empty_size):
      data_list[y * width + x] = -1
  rosmap.data = tuple(data_list)
  return rosmap

def convert_map(rosmap):
  """
    Convert a ros map object into a set of frontiers that can then be
    processed by other functions.
    See the file-level document for more information on what constitutes a
    frontier.
  """

  global pub_map
  global path_pub

  rospy.sleep(1.)
  # Pull the data out of the map.

  debug("Creating hole", False)
  # For testing purposes, create a hole in the map.
  #rosmap = create_hole(rosmap)
  data = rosmap.data
  grid_res = rosmap.info.resolution
  #print rosmap
  debug("Created hole", False)

  # We will iterate through every grid cell and determine whether it is on a
  # frontier at all; we will group things into separate frontiers later.
  frontier_nodes = set()
  width = rosmap.info.width
  height = rosmap.info.height
  for y in xrange(1, height):
    for x in xrange(1, width):
      val = data[y * width + x]
      if val != 0: continue # We only put unoccupied nodes on the frontier.
      # Check each of the 8 neighbors for -1.
      #top, left = data[(y - 1) * width +x], data[y * width + x - 1]
      #if top or left:
      #  print top, left
      if ((y and data[(y - 1) * width + x] == -1) or
          (x + 1 < width and data[y * width + x + 1] == -1) or
          (y + 1 < width and data[(y + 1) * width + x] == -1) or
          (x and data[y * width + x - 1] == -1)):
        frontier_nodes.add(Node(x, y))
  debug("Found Frontier nodes", False)

  # Now, go through and insert the graph structure.
  # Also, identify all the nodes that are either isolated or have just
  # one neighbor.
  isolated = set()
  end_nodes = set()
  for cur in frontier_nodes:
    for node in frontier_nodes:
      if node == cur:
        continue
      if abs(node.x - cur.x) <= 1 and abs(node.y - cur.y) <= 1:
        cur.neighbors.append(node)
    length = len(cur.neighbors)
    if length == 0:
      isolated.add(cur)
    elif length == 1:
      end_nodes.add(cur)
  #print frontier_nodes
  debug("Created Graph", False)

  temp_data = list(rosmap.data)
  for node in frontier_nodes:
    temp_data[width * node.y + node.x] = -50
  rosmap.data = tuple(temp_data)

  # Isolated nodes don't matter.
  frontier_nodes -= isolated
  debug("Colored Frontiers", False)

  # Now, we take nodes from isolated and expand them until we reach another
  # end_node.
  frontier_lists = []
  while len(end_nodes):
    debug("Creating new path: " + repr(end_nodes), False)
    start = end_nodes.pop()
    cur = start.neighbors[0]
    frontier = [start, cur]
    while cur not in end_nodes:
      neighbors = set(cur.neighbors)
      neighbors.remove(frontier[-2])
      if len(neighbors) == 0: break
      cur = neighbors.pop()
      added = True
      while cur in frontier:
        if len(neighbors):
          cur = neighbors.pop()
        else:
          added = False
          break
      if added: frontier.append(cur)
      else: break
    end_nodes.discard(cur)
    frontier_lists.append(frontier)

  while len(frontier_nodes) > 1:
    start = frontier_nodes.pop()
    cur = start.neighbors[0]
    ends = start.neighbors[1:]
    frontier = [start, cur]
    frontier_nodes.discard(start)
    frontier_nodes.discard(cur)
    while cur not in start.neighbors:
      neighbors = set(cur.neighbors)
      neighbors.remove(frontier[-2])
      if len(neighbors) == 0: break
      cur = neighbors.pop()
      frontier.append(cur)
      frontier_nodes.discard(cur)
    frontier_lists.append(frontier)

  debug("Done making lists", False)

  # Remove all nodes in frontier_lists from frontier_nodes.
  for frontier in frontier_lists:
    for node in frontier:
      frontier_nodes.discard(node)

  # We want to extract the centers of each list of nodes and drive to the
  # the closest.

  candidates = set(frontier_nodes)
  for frontier in frontier_lists:
    # If frontier is too small, ignore it.
    if len(frontier) > 5:
      median = int(len(frontier) / 2)
      candidates.add(frontier[median])

  # Now, convert everything into a PoseStamped and send them (in no order).
  msg = Path()
  msg.header.frame_id = "map"
  for node in candidates:
    pose = PoseStamped()
    pose.header.frame_id = "map"
    pose.pose.orientation.w = 1
    pose.pose.position.x = (node.x + 0.5) * grid_res + rosmap.info.origin.position.x
    pose.pose.position.y = (node.y + 0.5) * grid_res + rosmap.info.origin.position.y
    msg.poses.append(pose)

  path_pub.publish(msg)

  # Color various stuff in.
  temp_data = list(rosmap.data)
  for node in candidates:
    temp_data[width * node.y + node.x] = -100
  rosmap.data = tuple(temp_data)

  pub_map.publish(rosmap)
  debug("Published Map.", False)

  return # end convert_map()

if __name__ == '__main__':
  #global tf_list
  global pub_map
  rospy.init_node('frontier')
  #tf_list = tf.TransformListener()
  map_topic = rospy.get_param('~map_topic', '/map')
  rospy.Subscriber(map_topic, OccupancyGrid, convert_map, queue_size=1)
  pub_map = rospy.Publisher('/frontier_map', OccupancyGrid, queue_size=10)
  path_pub = rospy.Publisher('/frontier_path', Path, queue_size=10)
  rospy.spin()
