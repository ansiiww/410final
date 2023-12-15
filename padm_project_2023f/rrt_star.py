from random import random
import time
from motion_planner import *
import numpy as np
#   Parameters
#
#   Define the step size.  Also set a maximum number of nodes...
#
dstep = 1.5
Nmax  = 1000
r_one = 3
r_two = 1

RRT_ITERATIONS = 20
RED = (1, 0, 0)
INF = float('inf')

def apply_alpha(color, alpha=1.):
   return tuple(color[:3]) + (alpha,)

class TreeNode(object):

    def __init__(self, config, parent=None):
        self.config = config
        self.parent = parent

    def retrace(self):
        sequence = []
        node = self
        while node is not None:
            sequence.append(node)
            node = node.parent
        return sequence[::-1]

    def clear(self):
        self.node_handle = None
        self.edge_handle = None

    def __str__(self):
        return 'TreeNode(' + str(self.config) + ')'
    __repr__ = __str__


def irange(start, stop=None, step=1):  # np.arange
    if stop is None:
        stop = start
        start = 0
    while start < stop:
        yield start
        start += step

def argmin(function, sequence):
    # TODO: use min
    values = list(sequence)
    scores = [function(x) for x in values]
    return values[scores.index(min(scores))]

def configs(nodes):
    if nodes is None:
        return None
    return list(map(lambda n: n.config, nodes))

def elapsed_time(start_time):
    return time.time() - start_time

def nearestNbr(tree, fixedNode, radius, extend_fn, collision_fn):
    nearestNeighbors = []
    for node in tree:
        dist = node.distance(fixedNode)
        if dist <= radius and node.check_direct(node, fixedNode, extend_fn, collision_fn):
            nearestNeighbors.append(node)
    return nearestNeighbors

def rrt_star(start, goal_sample, distance_fn, sample_fn, extend_fn, collision_fn, goal_test=lambda q: False,
        goal_probability=.2, max_iterations=RRT_ITERATIONS, max_time=INF):
    """
    :param start: Start configuration - conf
    :param distance_fn: Distance function - distance_fn(q1, q2)->float
    :param sample_fn: Sample function - sample_fn()->conf
    :param extend_fn: Extension function - extend_fn(q1, q2)->[q', ..., q"]
    :param collision_fn: Collision function - collision_fn(q)->bool
    :param max_iterations: Maximum number of iterations - int
    :param max_time: Maximum runtime - float
    :return: Path [q', ..., q"] or None if unable to find a solution
    """
    start_time = time.time()


    if not callable(goal_sample):
        g = goal_sample
        goal_sample = lambda: g
    nodes = [TreeNode(start)] # this is the tree where we're adding everything to 
    for i in irange(max_iterations):
        if elapsed_time(start_time) >= max_time:
            break
        goal = random() < goal_probability or i == 0
        s = goal_sample() if goal else sample_fn() #nextnode

        last = argmin(lambda n: distance_fn(n.config, s), nodes)
        for q in extend_fn(last.config, s):
            # if collision_fn(q):
            #     break
            last = TreeNode(q, parent=last) #nearnode

            # computing the neighbors of last
            nearbyVertices_one = nearestNbr(nodes, s, r_one)
            nearbyVertices_two =  nearestNbr(nodes, s, r_two)

            vex_low = None #neightboring node with the lowest cost 
            near_costs = [] # list of the costs of neighbors

            for vex in nearbyVertices_one:
                near_costs.append(get_distance(vex, last)) # want to get the cost of this node 

            if len(near_costs) == 0:
                vex_low = last
            else: 
                vex_low_index = np.argmin(near_costs) 
                vex_low = nearbyVertices_one[vex_low_index] # neighbor node with the lowest cost
            
            s_cost = 0
            # Check whether to attach lowest cost node to the nextnode
            if check_direct(vex_low, s, extend_fn, collision_fn):

                # add to tree the connection between vex_low and s
                nodes.append(vex_low)
                s_cost = get_distance(s, vex_low) #there might need to be a vex_low cost but if we're using distance as cost......

                # branching out to check if any neighbors of the connection we 
                # just made are more efficient 
                # rewiring the tree to make more efficient connections
                for vex in nearbyVertices_two:
                    if get_distance(s, vex) < s_cost:
                        if check_direct(vex, s, extend_fn, collision_fn):
                            # add to tree
                            nodes.append(vex)

                # check if we've reached the goal node to connect to the goal node 
                if goal:
                    return configs(last.retrace())

        
    return None