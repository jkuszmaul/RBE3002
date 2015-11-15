#!/usr/bin/python
import math
import rospy
import inspect
import time
from nav_msgs.msg import Path, OccupancyGrid
from geometry_msgs.msg import Twist, Pose, PoseStamped, PoseWithCovarianceStamped
from std_msgs.msg import Empty

def debug(string):
  """
    Prints the name of the calling function and the line on which this was
    called, along with whatever string is passed.
  """
  return
  func = inspect.stack()[1]
  func_name = func[3]
  func_line = repr(func[2])
  print func_name + ":" + func_line + ": " + string

class PriorityQueue(object):
  """
    Class for managing the priority queue of nodes.
  """
  def __init__(self, first=None):
    debug("IN FRONTIER INIT.")
    self.head = first
    self.tail = self.head
    if self.head:
      self.head.cheaper = None
      self.head.expense = None

  def get(self):
    retval = self.head
    self.head = retval.expense
    retval.expense = None
    if self.head: self.head.cheaper = None
    return retval

  def empty(self):
    return self.head == None

  def update_queue(self, node):
    """
      If node is already in the queue, updates its position
      on the assumption that it can only move to the front of
      the queue.
      If not already in the queue, node is inserted.
    """
    debug(repr(node))
    current = self.tail
    if node.cheaper == None and node.expense == None:
      if self.head == None:
        # If empty, act specially.
        self.head = node
        self.tail = node
        return
      # insert in queue
      if (node > current):
        node.cheaper = current
        current.expense = node
        self.tail = node
        return
    else:
      # Start from current position in queue.
      current = node
    # Iterate down in the queue until we reach the right spot.
    while current.cheaper and node < current.cheaper:
      current = current.cheaper
    # Actually insert node into list.
    if node != current:
      debug(repr(node) + "; " + repr(current))
      # Tie up loose ends
      if (node == self.tail):
        self.tail = node.cheaper
      if node.expense:
        node.expense.cheaper = node.cheaper
      if node.cheaper:
        node.cheaper.expense = node.expense

      # Re-insert node
      node.cheaper = current.cheaper
      current.cheaper = node
      node.expense = current
      if node.cheaper: node.cheaper.expense = node
      else:
        self.head = node

class Node(object):
  """
    A node object which links to the identifiers for all adjacent nodes.
  """

  def __init__(self, x, y):
    # Identifiers should be a Node object.
    self.adj = set()
    self.parent = None
    # If in the frontier, this defines the next cheapest node.
    self.cheaper = None
    # If in the fronteir, this defines the next most expensive node.
    self.expense = None
    self.g_cost = -1
    self.h_cost = -1
    self.clear()
    self.x = x
    self.y = y

  def add_neighbor(self, neighbor):
    self.adj.add(neighbor)

  def rem_neighbor(self, neighbor):
    self.adj.remove(neighbor)

  def dist(self, node):
    return math.sqrt((node.y - self.y) ** 2 + (node.x - self.x) ** 2)

  def set_parent(self, parent):
    self.parent = parent

  def set_g(self, g):
    global frontier
    self.g_cost = g
    frontier.update_queue(self)

  def set_h(self, h):
    global frontier
    self.h_cost = h
    frontier.update_queue(self)

  def __repr__(self):
    x = repr(self.x)
    y = repr(self.y)
    cost = repr(self.g_cost + self.h_cost)
    total = "(" + x + ", " + y + ")" + " " + cost
    return total

  def __lt__(self, other):
    #debug("Comparing: " + repr(self.g_cost + self.h_cost) + " < " + repr(other.g_cost + other.h_cost))
    return (self.g_cost + self.h_cost) < (other.g_cost + other.h_cost)

  def clear(self):
    # Temporary variables for current instance.
    self.g_cost = float("inf")
    self.h_cost = float("inf")
    self.complete = False
    self.parent = None
    self.expense = None
    self.cheaper = None

class Map(object):
  """
    A class to represent the entire map to be used.
  """
  # nodelist is a two-dimensional list with each node being held
  # at its real-life coordinate. nodes that don't exist (eg, walls)
  # should node be adjacent to anything.
  nodelist = []

  def __init__(self, x, y, occupancy_arr):
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
        up_left = True # Whether or not to go for the node to the up-left.
        up_right = True # Whether or not to go for the node to the up-right.
        if i and self.nodelist[i - 1][j]:
          new_node.add_neighbor(self.nodelist[i - 1][j])
          self.nodelist[i-1][j].add_neighbor(new_node)
        else:
          # don't cut corners.
          up_left = False
          up_right = False
        if j and self.nodelist[i][j - 1]:
          new_node.add_neighbor(self.nodelist[i][j - 1])
          self.nodelist[i][j - 1].add_neighbor(new_node)
        else:
          # don't cut corners
          up_left = False
        # Check to see of the node to the right exists:
        if (occupancy_arr[x * i + j + 1] and (j + 1) < y) or (j + 1) >= y:
          up_right = False
        if up_left and self.nodelist[i - 1][j - 1]:
          new_node.add_neighbor(self.nodelist[i - 1][j - 1])
          self.nodelist[i - 1][j - 1].add_neighbor(new_node)
        if up_right and self.nodelist[i - 1][j + 1]:
          new_node.add_neighbor(self.nodelist[i - 1][j + 1])
          self.nodelist[i - 1][j + 1].add_neighbor(new_node)
        self.nodelist[i].append(new_node)
    #debug(repr(self.nodelist))

  def get_sl_dist(start, goal):
    return math.sqrt((goal[1] - start[1]) ** 2 + (goal[0] - start[0]) ** 2)

  def clear(self):
    for row in self.nodelist:
      for node in row:
        if node: node.clear()

  def get_node(self, xy):
    debug(repr(xy))
    return self.nodelist[xy[0]][xy[1]]

grid = None
grid_res = None
def convert_map(rosmap):
  """
    Takes a ros map message as in /map and converts it into a more
    useful representation in python.
    nav_msgs/OccupancyGrid
  """
  global grid
  global grid_res
  debug("converting map")
  grid = Map(rosmap.info.width, rosmap.info.height, rosmap.data)
  debug("map converted")
  # TODO: Convert between absolute coordinates and map.
  origin = rosmap.info.origin # Pose msg
  grid_res = rosmap.info.resolution

def get_grid_from_pose(pose):
  """
    Takes a ros Pose message and converts it to a grid coordinate.
    Limitations:
      -currently only accounts for resolution, not anything else.
  """
  x = pose.position.y / grid_res
  y = pose.position.x / grid_res
  return (int(x), int(y))

def pub_path(path):
  """
    Takes a list of nodes and publishes the path to the appropriate topic.
  """
  global path_pub
  msg = Path()
  msg.header.frame_id = "map"
  for pair in path:
    pose = PoseStamped()
    pose.header.frame_id = "map"
    pose.pose.orientation.w = 1
    pose.pose.position.x = (pair[1] + 0.5) * grid_res
    pose.pose.position.y = (pair[0] + 0.5) * grid_res
    msg.poses.append(pose)
  path_pub.publish(msg)

def print_queue():
  global frontier
  node = frontier.tail
  totalstr = ""
  i = 0
  while node:
    i += 1
    totalstr += repr(node)
    totalstr += "; "
    node = node.cheaper
  #print totalstr
  debug(repr(i))

def do_a_star(grid, start, goal):
  # Insert standard algorithm (see class notes, etc.).
  # Return appropriate path.
  global frontier
  grid.clear()
  frontier = PriorityQueue(start)
  closed = set()

  start.set_g(0)
  # Estimated total cost from start to goal through y.
  start.set_h(heuristic(start, goal))

  retval = False
  while not frontier.empty():
    current = frontier.get()
    #debug("Current: " + repr(current))
    if current == goal:
      retval = reconstruct_path(goal)
      break

    closed.add(current)

    for neighbor in current.adj:
      #debug("Neighbor: " + repr(neighbor))
      #print_queue()
      if neighbor in closed:
        continue    # Ignore the neighbor which is already evaluated.
      tentative_g_score = current.g_cost + current.dist(neighbor)
      #if neighbor.cheaper == None and neighbor.expense == None: # Discover a new node
      neighbor.set_h(heuristic(neighbor, goal))
      #print_queue()
      if tentative_g_score >= neighbor.g_cost:
        continue    # This is not a better path.

      # This path is the best until now. Record it!
      #debug("Best Path!")
      neighbor.parent = current
      neighbor.set_g(tentative_g_score)

  viz_data(grid, closed)
  if retval: pub_path(retval)
  viz_data(grid, closed)
  rospy.sleep(rospy.Duration(5, 0))

  return retval

def viz_data(grid, closed):
  # Visualize DATA
  global pub
  occ_msg = OccupancyGrid()
  occ_msg.info.resolution = grid_res
  occ_msg.info.width = len(grid.nodelist)
  occ_msg.info.height = len(grid.nodelist[0])
  occ_msg.info.origin.orientation.w = 1
  occ_data = [0] * (occ_msg.info.width * occ_msg.info.height)
  width = occ_msg.info.width
  height = occ_msg.info.height
  occ_data[width * start.x + start.y] = 100
  occ_data[width * goal.x + goal.y] = 100
  top_cost = 0
  for node in closed:
    if node.g_cost > top_cost: top_cost = node.g_cost
  for node in closed:
    cost = node.g_cost / top_cost
    cost *= 225
    if cost > 99: cost -= 228
    occ_data[width * node.x + node.y] = cost
  occ_msg.data = occ_data
  pub.publish(occ_msg)
  rospy.sleep(rospy.Duration(5, 0))


def do_stuff(empty):
  debug(repr(do_a_star(grid, start, goal)))

start = None
def get_start(start_pose):
  global start
  if grid: start = grid.get_node(get_grid_from_pose(start_pose.pose.pose))
  debug(repr(start))

goal = None
def get_goal(goal_pose):
  global goal
  if grid: goal = grid.get_node(get_grid_from_pose(goal_pose.pose))
  debug(repr(goal))
  do_stuff(1)

def reconstruct_path(current):
  total_path = [(current.x, current.y)]
  while current.parent:
    current = current.parent
    total_path.append((current.x, current.y))
  return total_path

def heuristic(start, goal):
  return start.dist(goal)

if __name__ == '__main__':
  global pub
  global path_pub
  rospy.init_node('astar')
  rospy.Subscriber('/map', OccupancyGrid, convert_map, queue_size=10)
  rospy.Subscriber('/move_base_simple/goal', PoseStamped, get_goal, queue_size=10)
  rospy.Subscriber('/initialpose', PoseWithCovarianceStamped, get_start, queue_size=10)
  rospy.Subscriber('/foobar', Empty, do_stuff, queue_size=1)
  pub = rospy.Publisher('/closed_nodes', OccupancyGrid, queue_size=10)
  path_pub = rospy.Publisher('/astar_path', Path, queue_size=10)
  rospy.spin()
