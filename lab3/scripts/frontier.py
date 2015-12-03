#!/usr/bin/python
import math
import numpy
import tf
import rospy
import inspect
import time
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

def convert_map(rosmap):
  """
    Convert a ros map object into a set of frontiers that can then be
    processed by other functions.
    See the file-level document for more information on what constitutes a
    frontier.
  """

  # Pull the data out of the map.
  data = rosmap.data
  grid_res = rosmap.info.resolution

  # We will iterate through every grid cell and determine whether it is on a
  # frontier at all; we will group things into separate frontiers later.
  frontier_nodes = set()
  width = rosmap.info.width
  height = rosmap.info.height
  for y in xrange(height):
    for x in xrange(width):
      val = data[y * width + x]
      if val != 0: continue # We only put unoccupied nodes on the frontier.
      # Check each of the 8 neighbors for -1.
      if (y and data[(y - 1) * width + x] == -1) or
         (x + 1 < width and data[y * width + x + 1] == -1) or
         (y + 1 < width and data[(y + 1) * width + x] == -1) or
         (x and data[y * width + x - 1] == -1):
        frontier_nodes.add(Node(x, y))

  # Now, go through and insert the graph structure.
  # Also, identify all the nodes that are either isolated or have just
  # one neighbor.
  isolated = set()
  end_nodes = set()
  for cur in frontier_nodes:
    for node in frontier_nodes:
      if abs(node.x - cur.x) <= 1 and abs(node.y - cur.y) <= 1:
        cur.neighbors.append(node)
    length = len(cur.neighbors)
    if length == 0:
      isolated.add(cur)
    elif length == 1:
      end_nodes.add(cur)
  # Isolated nodes don't matter.
  frontier_nodes -= isolated

  # Now, we take nodes from isolated and expand them until we reach another
  # end_node.
  frontier_lists = []
  while len(end_nodes):
    start = end_nodes.pop()
    cur = start.neighbors[0]
    frontier = [start, cur]
    while cur not in end_nodes:
      new_neighbors = set(cur.neighbors)
      new_neighbors.remove(cur)
      cur = new_neighbors.pop()
      frontier.append(cur)

if __name__ == '__main__':
  rospy.init_node('frontier')
  tf_list = tf.TransformListener()
  map_topic = rospy.get_param('~map_topic', '/map')
  rospy.Subscriber(map_topic, OccupancyGrid, convert_map, queue_size=1)
