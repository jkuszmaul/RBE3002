#!/usr/bin/python
import math
from Queue import PriorityQueue
from nav_msgs.msg import Path, OccupancyGrid
from geometry_msgs.msg import Twist, PoseStamped, Pose

class Node(object):
  """
    A node object which links to the identifiers for all adjacent nodes.
  """
  # Identifiers should be a Node object.
  adj = {}
  parent = None
  g_cost = -1
  h_cost = -1

  def __init__(self, x, y):
    self.clear()
    self.x = x
    self.y = y

  def add_neighbor(self, neighbor):
    adj.insert(neighbor)

  def rem_neighbor(self, neighbor):
    adj.remove(neighbor)

  def dist(self, node):
    return math.sqrt((node.y - self.y) ** 2 + (node.x - self.x) ** 2)

  def set_parent(self, parent):
    self.parent = parent

  def __lt__(self, other):
    return (self.g_cost + self.h_cost) < (other.g_cost + other.h_cost)

  def clear(self):
    # Temporary variables for current instance.
    self.total_cost = float("inf")
    self.g_cost = float("inf")
    self.complete = False
    self.parent = None

class Map(object):
  """
    A class to represent the entire map to be used.
  """
  # nodelist is a two-dimensional list with each node being held
  # at its real-life coordinate. nodes that don't exist (eg, walls)
  # should node be adjacent to anything.
  nodelist = []

  def __init__(self, x, y, occupancy_arr):
    # TODO: break out into getting sizes from map metadata.
    self.nodelist = []
    for i in xrange(x):
      self.nodelist.append([])
      for j in xrange(y):
        occ = occupancy_arr[x * i + j]
        if occ:
          self.nodelist[i].append(None)
          continue
        new_node = Node(i, j)
        # Add neighbor nodes that already exist.
        if i and self.nodelist[i - 1][j]:
          new_node.add_neighbor(self.nodelist[i - 1][j])
          self.nodelist[i-1][j].add_neighbor(new_node)
        if j and self.nodelist[i][j - 1]:
          new_node.add_neighbor(self.nodelist[i][j - 1])
          self.nodelist[i][j - 1].add_neighbor(new_node)
        self.nodelist[i].append(new_node)

  def get_sl_dist(start, goal):
    return math.sqrt((goal[1] - start[1]) ** 2 + (goal[0] - start[0]) ** 2)

  def clear(self):
    for row in nodelist:
      for node in row:
        node.clear()

  def get_node(self, nodeid):
    return nodelist[nodeid[0]][nodeid[1]]

grid = None
def convert_map(rosmap):
  """
    Takes a ros map message as in /map and converts it into a more
    useful representation in python.
    nav_msgs/OccupancyGrid
  """
  grid = Map(rosmap.width, rosmap.height, rosmap.data)
  # TODO: Convert between absolute coordinates and map.
  origin = rosmap.origin # Pose msg
  res = rosmap.resolution

def pub_path(path):
  """
    Takes a list of nodes and publishes the path to the appropriate topic.
  """

def do_a_star(grid, start, goal):
  # Insert standard algorithm (see class notes, etc.).
  # Return appropriate path.
  closed = {}
  frontier = PriorityQueue()
  frontier.put(start)

  start.g_cost = 0
  # Estimated total cost from start to goal through y.
  start.f_cost = heuristic(start, goal)

  while not frontier.empty():
    current = frontier.get()
    if current = goal:
      return reconstruct_path(goal)

    closed.insert(current)
    for neighbor in current.adj
      if neighbor in closed
        continue    # Ignore the neighbor which is already evaluated.
      tentative_g_score = current.g_cost + current.dist(neighbor)
      if neighbor not in frontier  # Discover a new node
        frontier.put(neighbor)
      else if tentative_g_score >= neighbor.g_cost
        continue    # This is not a better path.

      # This path is the best until now. Record it!
      neighbor.parent = current
      neighbor.g_cost = tentative_g_score

  return False

start = None
def get_start(start):

def reconstruct_path(current)
  total_path = [current]
  while current.parent:
    current = current.parent
    total_path.append(current)
  return total_path

def heuristic(start, goal):
  return start.dist(goal)

if __name__ == '__main__':
  rospy.init_node('astar')
  rospy.Subscriber('/map', OccupancyGrid, convert_map, queue_size=10)
  rospy.Subscriber('/move_base_simple/goal', PoseStamped, get_goal, queue_size=10)
  rospy.Subscriber('/initialpose', PoseStamped, get_start, queue_size=10)
