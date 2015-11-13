#!/usr/bin/python
import math

class Node(object):
  """
    A node object which links to the identifiers for all adjacent nodes.
  """
  # Identifiers should be a coordinate tuple (x, y)
  adj = []

  def __init__(self):
    self.clear()

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

  def __init__(self):
    # TODO: break out into getting sizes from map metadata.
    x = 200
    y = 200
    self.nodelist = []
    for i in xrange(x):
      self.nodelist.append([])
      for j in xrange(j):
        self.nodelist[i].append(Node())

  def get_sl_dist(start, goal):
    return math.sqrt((goal[1] - start[1]) ** 2 + (goal[0] - start[0]) ** 2)

  def clear(self):
    for row in nodelist:
      for node in row:
        node.clear()

  def get_node(self, nodeid):
    return nodelist[nodeid[0]][nodeid[1]]

def convert_map(rosmap):
  """
    Takes a ros map message as in /map and converts it into a more
    useful representation in python.
  """

def pub_path(path):
  """
    Takes a list of nodes and publishes the path to the appropriate topic.
  """

def do_a_star(grid, start, goal):
  # Insert standard algorithm (see class notes, etc.).
  # Return appropriate path.
