#!/usr/bin/python
import math
import traceback
import numpy
import tf
import rospy
import inspect
import time
from nav_msgs.msg import Path, OccupancyGrid
from nav_msgs.srv import GetPlan
from geometry_msgs.msg import Twist, Pose, PoseStamped, PoseWithCovarianceStamped
from std_msgs.msg import Empty

def debug(string, disabled=True):
  """
    Prints the name of the calling function and the line on which this was
    called, along with whatever string is passed.
  """
  if disabled: return
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

  def __init__(self, x, y, cost=0):
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
    self.cost = cost

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
    for i in xrange(y):
      self.nodelist.append([])
      for j in xrange(x):
        occ = occupancy_arr[x * i + j]
        if occ == 100 or occ == -1:
          self.nodelist[i].append(None)
          continue
        cost = 1000 if occ == 99 else occ / 10.
        new_node = Node(i, j, cost=cost)
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
        if ((j + 1) < x and occupancy_arr[x * i + j + 1]) or (j + 1) >= x:
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

def get_surround(data, node, dist, width, height):
  x = node[0]
  y = node[1]
  #width, height = height, width
  if data[y * width + x]:
    return data[y * width + x]
  startx = x - dist
  startx = 0 if startx < 0 else startx
  endx = x + dist
  endx = width-1 if endx >= width else endx
  starty = y - dist
  starty = 0 if starty < 0 else starty
  endy = y + dist
  endy = height-1 if endy >= height else endy
  cells = []
  for i in range(starty, endy + 1):
    for j in range(startx, endx + 1):
      val = data[i * width + j]
      if val == 100:
        return 100
      #if val > 0:#data[i * width + j]:# > 0:
      #  return val
      #if val == -1:
      #  if i == x and j == y:
      #    return -1
  return 0

grid = None
grid_res = None
map_lock = False
def convert_map(rosmap):
  """
    Takes a ros map message as in /map and converts it into a more
    useful representation in python.
    nav_msgs/OccupancyGrid
  """
  global grid
  global grid_res
  global grid_frame, grid_transform
  global map_lock
  global tf_list
  global pub
  debug("converting map")
  grid_res = rosmap.info.resolution

  # Get map origin in coordinates of the '/map' tf frame.
  grid_zero = rosmap.info.origin
  grid_frame = rosmap.header.frame_id
  grid_stamped = PoseStamped()
  grid_stamped.pose = grid_zero
  grid_stamped.header.frame_id = rosmap.header.frame_id
  transform_pose = tf_list.transformPose('map', grid_stamped).pose
  position = transform_pose.position
  orientation = transform_pose.orientation
  new_grid_transform = tf.transformations.quaternion_matrix(
      [orientation.x, orientation.y, orientation.z, orientation.w])
  new_grid_transform[0, 3] = position.x
  new_grid_transform[1, 3] = position.y
  new_grid_transform[2, 3] = position.z
  #grid_zero_transform = numpy.array([[1, 0, 0, grid_zero.position.x],
  #                                   [0, 1, 0, grid_zero.position.y],
  #                                   [0, 0, 1, 0],
  #                                   [0, 0, 0, 1]])
  #grid_transform = numpy.dot(grid_transform, grid_zero_transform)

  # Go through and put clearance around the walls.
  data = rosmap.data
  new_data = []
  diff = 4#int((.4 / 2) / grid_res) + 1 # Add one to provide buffer
  width = rosmap.info.width
  height = rosmap.info.height
  if rospy.get_param('~obstacle_expansion', True):
    print "EXPANDING OBSTACLES"
    for y in xrange(height):
      for x in xrange(width):
        new_data.append(get_surround(data, (x, y), diff, width, height))
  else: new_data = data
  #rosmap.data = new_data
  #rospy.sleep(10.)
  #pub.publish(rosmap)
  new_grid = Map(rosmap.info.width, rosmap.info.height, new_data)

  # Perform actual update of grid.
  if map_lock:
    while map_lock: continue
  map_lock = True
  grid_transform = new_grid_transform
  grid = new_grid
  map_lock = False


  debug("map converted")
  # TODO: Convert between absolute coordinates and map.
  origin = rosmap.info.origin # Pose msg

def get_grid_from_pose(pose):
  """
    Takes a ros PoseStamped message and converts it to a grid coordinate.
  """
  global tf_list, grid_frame, grid_transform
  pose = pose.pose.position
  pose_vec = numpy.array([[pose.x],[pose.y],[pose.z],[1]])
  pose = numpy.dot(numpy.linalg.inv(grid_transform), pose_vec)
  x = pose[0, 0] / grid_res
  y = pose[1, 0] / grid_res
  return (int(y), int(x))

def get_waypoints(path):
  """
    Simplify a path down into just the turns.
  """
  global grid_res
  if len(path) < 2:
    return path
  max_dist = 1.5
  waypoints = [path[0]]
  cur = path[1]
  prev_diff = (cur[0] - path[0][0], cur[1] - path[0][1])
  for i in range(1, len(path) - 1):
    cur = path[i]
    new = path[i + 1]
    diff = (new[0] - cur[0], new[1] - cur[1])
    dist = math.sqrt((cur[0] - waypoints[-1][0]) ** 2 +
                     (cur[1] - waypoints[-1][1]) ** 2) * grid_res
    if diff != prev_diff or dist > max_dist:
      waypoints.append(cur)
    prev_diff = diff

  waypoints.append(path[-1])
  return waypoints

def pub_path(path):
  """
    Takes a list of nodes and publishes the path to the appropriate topic.
  """
  global path_pub
  global grid_res, grid_frame
  msg = Path()
  msg.header.frame_id = grid_frame
  for pair in path:
    pose = PoseStamped()
    pose.header.frame_id = grid_frame
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

def get_turn_cost(start, neighbor):
  if start.parent == None:
    return 0
  prev = start.parent
  start_diff = (start.x - prev.x, start.y - prev.y)
  next_diff = (neighbor.x - start.x, neighbor.y - start.y)
  if start_diff == next_diff:
    return 0
  elif start_diff[0] == next_diff[0] or start_diff[1] == next_diff[1]:
    return 1
  else:
    return 2

def do_a_star(grid, start, goal):
  # Insert standard algorithm (see class notes, etc.).
  # Return appropriate path.
  if goal == None:
    return []
  if start == goal:
    return [(start.x, start.y)]
  global frontier
  grid.clear()
  frontier = PriorityQueue(start)
  closed = set()

  start.set_g(0)
  # Estimated total cost from start to goal through y.
  start.set_h(heuristic(start, goal))

  retval = []
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
      tentative_g_score = (current.g_cost + current.dist(neighbor)
          + get_turn_cost(current, neighbor) + neighbor.cost)
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
  if retval: pub_path(get_waypoints(retval))

  return retval

def viz_data(grid, closed):
  # Visualize DATA
  global pub
  global grid_res, grid_transform
  occ_msg = OccupancyGrid()
  occ_msg.info.resolution = grid_res
  occ_msg.info.width = len(grid.nodelist)
  occ_msg.info.height = len(grid.nodelist[0])
  occ_msg.header.frame_id = "map"
  occ_msg.info.origin.position.x = grid_transform[0, 3]
  occ_msg.info.origin.position.y = grid_transform[1, 3]
  quaternion = tf.transformations.quaternion_from_matrix(grid_transform)
  occ_msg.info.origin.orientation.x = quaternion[0]
  occ_msg.info.origin.orientation.y = quaternion[1]
  occ_msg.info.origin.orientation.z = quaternion[2]
  occ_msg.info.origin.orientation.w = quaternion[3]
  occ_data = [0] * (occ_msg.info.width * occ_msg.info.height)
  width = occ_msg.info.width
  height = occ_msg.info.height
  #occ_data[width * start.x + start.y] = 100
  #occ_data[width * goal.x + goal.y] = 100
  top_cost = 1
  for node in closed:
    if node.g_cost > top_cost: top_cost = node.g_cost
  for node in closed:
    cost = node.g_cost / top_cost
    cost *= 225
    if cost > 99: cost -= 228
    occ_data[width * node.x + node.y] = cost
  occ_msg.data = occ_data
  pub.publish(occ_msg)

def astar_serv(info):
  global tf_list, grid_transform
  global map_lock
  while map_lock: continue
  while not grid: continue
  map_lock = True
  path = []
  start, goal = None, None
  try:
    start = get_grid_from_pose(info.start)
    goal = get_grid_from_pose(info.goal)
    if grid: start = grid.get_node(start)
    if grid: goal = grid.get_node(goal)
    print "Got start/goal:", start, goal
  except Exception as exc:
    print exc
    traceback.print_exc()
  try:
    if start and goal:
      print "Doing Calculations!"
      path = get_waypoints(do_a_star(grid, start, goal))
  finally:
    map_lock = False
  msg = Path()
  msg.header.frame_id = "map"
  for i in xrange(len(path)):
    pair = path[i]
    next_pair = (0, 0)
    if i - 1 >= 0: next_pair = path[i - 1]
    pose = PoseStamped()
    pose.header.frame_id = "map"
    pos_theta = math.atan2(next_pair[0] - pair[0], next_pair[1] - pair[1])
    quat = tf.transformations.quaternion_from_euler(0, 0, pos_theta)
    pose.pose.orientation.x = quat[0]
    pose.pose.orientation.y = quat[1]
    pose.pose.orientation.z = quat[2]
    pose.pose.orientation.w = quat[3]
    pos_vec = numpy.array([[(pair[1] + 0.5) * grid_res],
                           [(pair[0] + 0.5) * grid_res],
                           [0],
                           [1]])
    pos_vec = numpy.dot(grid_transform, pos_vec)
    pose.pose.position.x = pos_vec[0]
    pose.pose.position.y = pos_vec[1]
    msg.poses.append(pose)
  return msg

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
  global tf_list
  rospy.init_node('astar')
  map_topic = rospy.get_param('~map_topic', '/map')
  rospy.Subscriber(map_topic, OccupancyGrid, convert_map, queue_size=1)
  tf_list = tf.TransformListener()
  cost_out_topic = rospy.get_param('~cost_out', '/closed_nodes')
  pub = rospy.Publisher(cost_out_topic, OccupancyGrid, queue_size=10)
  path_pub = rospy.Publisher('/astar_path', Path, queue_size=10) # unused
  service_name = rospy.get_param('~service_name', 'astar')
  path_serv = rospy.Service(service_name, GetPlan, astar_serv)
  rospy.spin()
