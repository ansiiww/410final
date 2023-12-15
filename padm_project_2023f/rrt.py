from random import random
import time
import numpy as np

RRT_ITERATIONS = 20
RED = (1, 0, 0)
INF = float('inf')

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

def rrt(start, goal_sample, distance_fn, sample_fn, extend_fn, collision_fn, goal_test=lambda q: False,
        goal_probability=.1, max_iterations=RRT_ITERATIONS, max_time=INF):
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
    # if collision_fn(start):
    #     return None
    if not callable(goal_sample):
        g = goal_sample
        goal_sample = lambda: g
    nodes = [TreeNode(start)]
    print("make sure restarts", nodes)
    for i in irange(max_iterations):
    # for i in np.arange(20):
        if elapsed_time(start_time) >= max_time:
            break
        goal = random() < goal_probability or i == -1
        s = goal_sample() if goal else sample_fn()
        print(goal, "GOAL BIAS", i, goal_sample(), s)

        last = argmin(lambda n: distance_fn(n.config, s), nodes)
        # print(last.config)
        for q in extend_fn(last.config, s):
            if collision_fn(q):
                print("collision", q, s)
                break
            last = TreeNode(q, parent=last)
            nodes.append(last)
            if goal_test(last.config):
                return configs(last.retrace())
        else:
            if goal:
                return configs(last.retrace())
    return None
