import time

from rrt import rrt, INF, elapsed_time
from rrt_star import rrt_star
from collections import deque

def bisect(sequence):
    sequence = list(sequence)
    indices = set()
    queue = deque([(0, len(sequence)-1)])
    while queue:
        lower, higher = queue.popleft()
        if lower > higher:
            continue
        index = int((lower + higher) / 2.)
        assert index not in indices
        yield sequence[index]
        queue.extend([
            (lower, index-1),
            (index+1, higher),
        ])

def bisect_selector(path):
    return bisect(path)

default_selector = bisect_selector

def is_path(path):
    return path is not None

def get_pairs(sequence):
    sequence = list(sequence)
    return list(zip(sequence[:-1], sequence[1:]))

def direct_path(start, goal, extend_fn, collision_fn):
    """
    :param start: Start configuration - conf
    :param goal: End configuration - conf
    :param extend_fn: Extension function - extend_fn(q1, q2)->[q', ..., q"]
    :param collision_fn: Collision function - collision_fn(q)->bool
    :return: Path [q', ..., q"] or None if unable to find a solution
    """
    # TODO: version which checks whether the segment is valid
    if collision_fn(start) or collision_fn(goal):
        # TODO: return False
        print('checked here')
        return None
    path = list(extend_fn(start, goal))
    path = [start] + path
    if any(collision_fn(q) for q in default_selector(path)):
        return None
    return path


def check_direct(start, goal, extend_fn, collision_fn):
    if any(collision_fn(q) for q in [start, goal]):
        return None
    return direct_path(start, goal, extend_fn, collision_fn)

#################################################################
def solve(start, goal, distance_fn, sample_fn, extend_fn, collision_fn, algs = 'rrt',
          max_time=INF, max_iterations=INF, num_samples=100, **kwargs):

    start_time = time.time()
    # path = check_direct(start, goal, extend_fn, collision_fn)
    path = None
    if (path is not None and path is not False):
        print("direct path returned======================================", start, goal, path)
        return path

    remaining_time = max_time - elapsed_time(start_time)
    if algs == 'rrt':
        print('rrt')
        path = rrt(start, goal, distance_fn, sample_fn, extend_fn, collision_fn,
                    max_iterations=max_iterations, max_time=remaining_time)
    elif algs == 'rrt_star':
        print('rrt_star')
        path = rrt_star(start, goal, distance_fn, sample_fn, extend_fn, collision_fn, radius=0.1,
                        max_iterations=max_iterations, max_time=remaining_time)
        
    end_time = time.time()
    print(start_time, end_time, end_time-start_time) 
    print("searched path returned======================================", start, goal, path)
    return path



 
