import time

from rrt import rrt, INF, elapsed_time
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
        #if is_even(higher - lower):
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

# def get_distance(q1, q2):
#     return np.linalg.norm(get_delta(q1, q2))

# def compute_path_cost(path, cost_fn=get_distance):
#     if not is_path(path):
#         return INF
#     return sum(cost_fn(*pair) for pair in get_pairs(path))

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
def solve(start, goal, distance_fn, sample_fn, extend_fn, collision_fn,
          max_time=INF, max_iterations=INF, num_samples=100, **kwargs):

    start_time = time.time()
    # path = check_direct(start, goal, extend_fn, collision_fn)
    path = None
    if (path is not None and path is not False):
        print("direct path returned======================================", start, goal, path)
        return path

    remaining_time = max_time - elapsed_time(start_time)

    path = rrt(start, goal, distance_fn, sample_fn, extend_fn, collision_fn,
                max_iterations=max_iterations, max_time=remaining_time)
    print("rrt path returned======================================", start, goal, path)
    return path



 