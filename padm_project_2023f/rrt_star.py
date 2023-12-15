from __future__ import print_function

from random import random
import time

RED = (1, 0, 0)
INF = float('inf')

EPSILON = 1e-6
PRINT_FREQUENCY = 100

class OptimalNode(object):

    def __init__(self, config, parent=None, d=0, path=[], iteration=None):
        self.config = config
        self.parent = parent
        self.children = set()
        self.d = d
        self.path = path
        if parent is not None:
            self.cost = parent.cost + d
            self.parent.children.add(self)
        else:
            self.cost = d
        self.solution = False
        self.creation = iteration
        self.last_rewire = iteration

    def set_solution(self, solution):
        if self.solution is solution:
            return
        self.solution = solution
        if self.parent is not None:
            self.parent.set_solution(solution)

    def retrace(self):
        if self.parent is None:
            return self.path + [self.config]
        return self.parent.retrace() + self.path + [self.config]

    def rewire(self, parent, d, path, iteration=None):
        if self.solution:
            self.parent.set_solution(False)
        self.parent.children.remove(self)
        self.parent = parent
        self.parent.children.add(self)
        if self.solution:
            self.parent.set_solution(True)
        self.d = d
        self.path = path
        self.update()
        self.last_rewire = iteration

    def update(self):
        self.cost = self.parent.cost + self.d
        for n in self.children:
            n.update()

    def __str__(self):
        return self.__class__.__name__ + '(' + str(self.config) + ')'
    __repr__ = __str__


def safe_path(sequence, collision):
    path = []
    for q in sequence:
        if collision(q):
            break
        path.append(q)
    return path

def elapsed_time(start_time):
    return time.time() - start_time

##################################################
def argmin(function, sequence):
    values = list(sequence)
    scores = [function(x) for x in values]
    return values[scores.index(min(scores))]

def rrt_star(start, goal, distance_fn, sample_fn, extend_fn, collision_fn, radius,
             max_time=INF, max_iterations=INF, goal_probability=.2):
    """
    :param start: Start configuration - conf
    :param goal: End configuration - conf
    :param distance_fn: Distance function - distance_fn(q1, q2)->float
    :param sample_fn: Sample function - sample_fn()->conf
    :param extend_fn: Extension function - extend_fn(q1, q2)->[q', ..., q"]
    :param collision_fn: Collision function - collision_fn(q)->bool
    :param max_time: Maximum runtime - float
    :return: Path [q', ..., q"] or None if unable to find a solution
    """
    # if collision_fn(start) or collision_fn(goal):
    #     return None
    nodes = [OptimalNode(start)]
    goal_n = None
    start_time = time.time()
    iteration = 0
    while (elapsed_time(start_time) < max_time) and (iteration < max_iterations):
        do_goal = goal_n is None and (iteration == 0 or random() < goal_probability)
        s = goal if do_goal else sample_fn()

        if iteration % PRINT_FREQUENCY == 0:
            success = goal_n is not None
            cost = goal_n.cost if success else INF
            print('Iteration: {} | Time: {:.3f} | Success: {} | {} | Cost: {:.3f}'.format(
                iteration, elapsed_time(start_time), success, do_goal, cost))
        iteration += 1

        nearest = argmin(lambda n: distance_fn(n.config, s), nodes)
        path = safe_path(extend_fn(nearest.config, s), collision_fn)
        if len(path) == 0:
            continue
        new = OptimalNode(path[-1], parent=nearest, d=distance_fn(
            nearest.config, path[-1]), path=path[:-1], iteration=iteration)
        # if safe and do_goal:
        if do_goal and (distance_fn(new.config, goal) < EPSILON):
            goal_n = new
            goal_n.set_solution(True)

        neighbors = filter(lambda n: distance_fn(n.config, new.config) < radius, nodes)
        nodes.append(new)

        for n in neighbors:
            d = distance_fn(n.config, new.config)
            if (n.cost + d) < new.cost:
                path = safe_path(extend_fn(n.config, new.config), collision_fn)
                if (len(path) != 0) and (distance_fn(new.config, path[-1]) < EPSILON):
                    new.rewire(n, d, path[:-1], iteration=iteration)
        for n in neighbors:  
            d = distance_fn(new.config, n.config)
            if (new.cost + d) < n.cost:
                path = safe_path(extend_fn(new.config, n.config), collision_fn)
                if (len(path) != 0) and (distance_fn(n.config, path[-1]) < EPSILON):
                    n.rewire(new, d, path[:-1], iteration=iteration)
        if goal_n is not None and iteration >20:
            print(iteration)
            return goal_n.retrace()
    if goal_n is None:
        return None
    return goal_n.retrace()
