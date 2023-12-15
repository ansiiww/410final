from __future__ import print_function

import os
import sys
import argparse
import numpy as np
import pybullet as p
import time


sys.path.extend(os.path.abspath(os.path.join(os.getcwd(), d)) for d in ['pddlstream', 'ss-pybullet'])

from pybullet_tools.utils import set_pose, Pose, Point, Euler, multiply, get_pose, get_point, create_box, set_all_static, WorldSaver, BodySaver, create_plane, COLOR_FROM_NAME, stable_z_on_aabb, pairwise_collision, elapsed_time, get_aabb_extent, get_aabb, create_cylinder, set_point, get_function_name, wait_for_user, dump_world, set_random_seed, set_numpy_seed, get_random_seed, get_numpy_seed, set_camera, set_camera_pose, link_from_name, get_movable_joints, get_joint_name, get_joint_positions
from pybullet_tools.utils import CIRCULAR_LIMITS, get_custom_limits, set_joint_positions, interval_generator, get_link_pose, interpolate_poses, check_initial_end, get_name, LockRenderer
from pybullet_tools.utils import get_base_name, get_body_name, link_pairs_collision, get_links, get_all_links, get_distance_fn, get_extend_fn, get_collision_fn, get_bodies
from pybullet_tools.utils import get_base_difference_fn, get_base_distance_fn, get_base_values, set_base_values, get_full_configuration, get_joints

from pybullet_tools.ikfast.franka_panda.ik import PANDA_INFO, FRANKA_URDF
from pybullet_tools.ikfast.ikfast import get_ik_joints, closest_inverse_kinematics


from src.world import World
from src.utils import JOINT_TEMPLATE, BLOCK_SIZES, BLOCK_COLORS, COUNTERS, \
    ALL_JOINTS, LEFT_CAMERA, CAMERA_MATRIX, CAMERA_POSES, CAMERAS, compute_surface_aabb, \
    BLOCK_TEMPLATE, name_from_type, GRASP_TYPES, SIDE_GRASP, joint_from_name, \
    STOVES, TOP_GRASP, randomize, LEFT_DOOR, point_from_pose, translate_linearly, \
    Grasp, get_grasps, get_obstacles\

import motion_planner

UNIT_POSE2D = (0., 0., 0.)
MAX_DISTANCE = 0.001

spam_base = [1.1, 1, np.pi]
spam_drop = [1, 1.2, np.pi]
drawer_base = [1.2, 1.2, np.pi]
left_base = [0.8, -.5, np.pi]
sugar_base = [0.74, 0.63, np.pi]
post_drawer_base = [1.4, 1.2, np.pi]
drop_base = [1.2, 0.63, np.pi]

pre_grip = [0, 0.35, 0, -1.5, 0, 2, 0.86]
sugar_grip = [0, 1.6, 0, -0.2, 0, 3, 0.86]
spam_grip = [0, 1.6, 0, -.25, 0, 3, 0.86]
# pre_handle_grip = [0, 1.6, 0, -.7, 0, 3.5, -0.2]
post_handle_grip = [0, 0.6, 0, -2.5, 0, 3.5, 0.86] #
pre_handle_grip = [0.11859342968775645, 1.7604774984574674, -0.31209484262994963, -0.19125243207142018, 0.5383624968804135, 3.0948436725448696, -0.24397344000986498]
#pre_handle_grip = [0, 1.6, 0, -.7, 0, 3.5, 4]


drop_high = [0, 0.35, 0, -1.5, 0, 2, 0.86] #
# drop_low = [0, 0.68, 0, -1.5, 0, 2, 0.86] #
drop_low = [0, 0.9, 0, -1.2, 0, 2, 0.86]

def add_ycb(world, ycb_type, idx=0, counter=0, **kwargs):
    name = name_from_type(ycb_type, idx)
    world.add_body(name, color=np.ones(4))
    pose2d_on_surface(world, name, COUNTERS[counter], **kwargs)
    return name

def pose2d_on_surface(world, entity_name, surface_name, pose2d=UNIT_POSE2D):
    x, y, yaw = pose2d
    body = world.get_body(entity_name)
    surface_aabb = compute_surface_aabb(world, surface_name)
    z = stable_z_on_aabb(body, surface_aabb)
    pose = Pose(Point(x, y, z), Euler(yaw=yaw))
    set_pose(body, pose)
    return pose

add_sugar_box = lambda world, **kwargs: add_ycb(world, 'sugar_box', **kwargs)
add_spam_box = lambda world, **kwargs: add_ycb(world, 'potted_meat_can', **kwargs)



def get_sample_fn(body, joints, custom_limits={}, **kwargs):
    lower_limits, upper_limits = get_custom_limits(body, joints, custom_limits, circular_limits=CIRCULAR_LIMITS)

    generator = interval_generator(lower_limits, upper_limits, **kwargs)
    def fn():
        return tuple(next(generator))
    return fn


def extract_act(act):
    action, loc, effectors = act.name.split('_')[0], [], []
    params = act.parameters
    
    param_mapping = {
        'b1': 'sugar_box0',
        'b2': 'potted_meat_can1',
        't1': 'spam_base',
        't2': 'sugar_base',
        't3': 'left_base',
        'c1': 'cabinet',
        'd1': 'drawer_base'
    }
    
    for p in params:
        if p in param_mapping:
            if p.startswith('b'):
                effectors.append(param_mapping[p])
            else:
                loc.append(param_mapping[p])
    
    return action, loc, effectors

def plan_base(act, world, algs):
    action, loc, obj = act
    print('THHIS IS THE ACTIVITY:', act)
    obj_body = ''
    location = ''
    if len(loc) != 0:
        location = loc[0]
    if len(obj) != 0:
        obj_body = world.body_from_name[obj[0]]
        
    if action == 'grip':
        (x,y,_) = get_pose(obj_body)[0]
        goal_pose = (x+0.94, y-0.02, 3.14)
    elif action == 'drop':
        if location == 'left_base':
            goal_pose = left_base
        elif location == 'spam_base':
            goal_pose = spam_drop
        elif location == 'sugar_base':
            goal_pose = sugar_base
        elif location == 'drawer_base':
            goal_pose = spam_drop #drawer_base
            print('Im here!$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$')
        else:
            goal_pose = post_drawer_base
    elif action == 'open':
        goal_pose = drawer_base
    elif action == 'move_drawer':
        goal_pose = [1.5, 1.2, np.pi]

    init_conf = get_joint_positions(world.robot, world.base_joints)
    base_limits = ((0.74, -1), (2, 1))

    print('THIS IS THE GOAL POSE', goal_pose)

    with LockRenderer():    
        bestPath = plan_base_motion(world, world.robot, init_conf, goal_pose, base_limits,
                                    obstacles=world.static_obstacles, algs=algs)

    print(init_conf, goal_pose, bestPath)                        
    return bestPath

def plan_base_motion(world, body, start_conf, end_conf, base_limits, obstacles=[], algs='rrt', direct=False,
                     weights=1*np.ones(3), resolutions=0.05*np.ones(3),
                     max_distance=0.001, **kwargs):
    def sample_fn():
        x, y = np.random.uniform(*base_limits)
        theta = np.random.uniform(*CIRCULAR_LIMITS)
        theta = normalize_angle(theta)
        return (x, y, theta)

    difference_fn = get_base_difference_fn()
    distance_fn = get_base_distance_fn(weights=weights)

    def extend_fn(q1, q2):
        max_step = 0.3
        if distance_fn(q2, q1) > max_step:
            steps = np.abs(np.divide(difference_fn(q2, q1), resolutions))
            n = int(np.max(steps)) + 1
            q = q1
            for i in range(n):
                q = tuple((1. / (n - i)) * np.array(difference_fn(q2, q)) + q)
                yield q
        else:
            return q2

    def collision_fn(q):
        # TODO: update this function
        # set_base_values(body, q)
        set_joint_positions(body, world.base_joints, q)
        for obs in obstacles:
            if pairwise_collision(body, obs, max_distance=max_distance):
                return True
    
    if direct:
        from motion_planner import direct_path
        path = direct_path(start_conf, end_conf, extend_fn, collision_fn)
        return path

    from motion_planner import solve
    print('BASE ALGS', algs)
    path = solve(start_conf, end_conf, distance_fn, sample_fn, extend_fn, collision_fn, algs=algs, **kwargs)
    return path

def plan_arm(act, world, attachments=[], algs = 'rrt'):
    print(act)
    action, loc, obj = act
    obj_body = ''
    location = ''
    if len(obj) != 0:
        obj_body = obj[0]
    if len(loc) != 0:
        location = loc[0]

    goal_pose = pre_grip
    if action == 'pre_grip_drawer':
        goal_pose = pre_handle_grip
    elif action == 'pre_grip':
        goal_pose = pre_grip
    elif action == 'grip':
        if obj_body == 'sugar_box0':
            goal_pose = sugar_grip
        elif obj_body == 'potted_meat_can1':
            goal_pose = spam_grip
    elif action == 'drop':
        if location == 'left_base':
            goal_pose = sugar_grip
        elif location == 'spam_base':
            goal_pose = sugar_grip
        elif location == 'sugar_base':
            goal_pose = drop_high
        else:
            goal_pose = drop_low
    elif action == 'open':
        goal_pose = post_handle_grip
    
    print(action, goal_pose, obj_body)
    start_conf = get_joint_positions(world.robot, world.arm_joints)

    # breakpoint()
    with LockRenderer():
        bestPath = plan_joint_motion(world.robot, world.arm_joints, goal_pose, obstacles=world.static_obstacles, attachments=attachments, algs = algs) 
    
    set_joint_positions(world.robot, world.arm_joints,start_conf)

    return bestPath


def plan_joint_motion(body, joints, end_conf, obstacles=[], attachments=[], algs = 'rrt',
                      self_collisions=True, disabled_collisions=set(),
                      weights=None, resolutions=None, max_distance=MAX_DISTANCE,
                      use_aabb=False, cache=True, custom_limits={}, **kwargs):

    assert len(joints) == len(end_conf)
    if (weights is None) and (resolutions is not None):
        weights = np.reciprocal(resolutions)
    distance_fn = get_distance_fn(body, joints, weights=weights)
    sample_fn = get_sample_fn(body, joints, custom_limits=custom_limits)

    collision_fn = get_collision_fn(body, joints, obstacles, attachments, self_collisions, disabled_collisions,
                                    custom_limits=custom_limits, max_distance=max_distance,
                                    use_aabb=use_aabb, cache=cache)
    


    extend_fn = get_extend_fn(body, joints, resolutions=resolutions)
    start_conf = get_joint_positions(body, joints)

    from motion_planner import solve
    return solve(start_conf, end_conf, distance_fn, sample_fn, extend_fn, collision_fn, algs, **kwargs)
    
def normalize_angle(angle):
    """Normalize an angle to the range [-pi, pi]."""
    while angle > np.pi:
        angle -= 2 * np.pi
    while angle < -np.pi:
        angle += 2 * np.pi
    return angle
    
def execute_act(act, world, id=0, algs='rrt'):
    print(act)
    tool_link = link_from_name(world.robot, 'panda_hand')

    action, loc, obj = act
    gl = []
    grasps = None
    choose_grasp = None
    obj_body = ''
    if len(obj) != 0:
        obj_body = obj[0]
    if obj_body == 'sugar_box0':
        grasps = get_grasps(world, obj_body, [SIDE_GRASP])
        for grasp in grasps:
            gl.append(grasp)
    elif obj_body == 'potted_meat_can1':
        grasps = get_grasps(world, obj_body, [TOP_GRASP])
        for grasp in grasps:
            gl.append(grasp)
        
    if len(gl) != 0:
        choose_grasp = gl[0]
    


    # breakpoint()
    pre_path = plan_arm(('pre_grip', [], []), world, algs=algs)
    # breakpoint()
    time.sleep(0.5)
    executeArm(pre_path, world, action, tool_link, choose_grasp, gl, 'pre', id)
    print(pre_path)

    # breakpoint()
    time.sleep(0.5)

    base_path = plan_base(act, world, algs=algs)
    set_joint_positions(world.robot, world.base_joints, base_path[0])
    time.sleep(0.5)

    
    print("base plan", base_path)
    
    init_base_pos = get_joint_positions(world.robot, world.base_joints)
    (x, y, theta) = init_base_pos
    act_path = []
    # base_path.insert(0, init_base_pos)
    for next_pos in base_path[1:]:
        (new_x, new_y, new_theta) = next_pos
        new_theta = normalize_angle(new_theta)
        dx = new_x - x
        dy = new_y - y
        angle = np.arctan2(dy, dx)
        dangle = angle-theta
        step = 0
        if np.abs(dangle) > np.pi/4 :
            step = 100
        else:
            step = 10
        for i in range (step):
            if action == 'drop' and id != 0:
                choose_grasp.assign()
            set_joint_positions(world.robot, world.base_joints, (x,y,theta+dangle/step))
            time.sleep(0.0001*step)
            theta += dangle/step
        for i in range(100):
            distance = np.sqrt(dx**2 + dy**2)
            goal_pos =translate_linearly(world, distance=distance/100)
            if action == 'drop' and id != 0:
                choose_grasp.assign()
            set_joint_positions(world.robot, world.base_joints, goal_pos)
            time.sleep(0.001)

        (x, y, theta) = get_joint_positions(world.robot, world.base_joints)
        act_path.append((x, y, theta))
        theta = normalize_angle(theta)

    (x, y, new_theta) = base_path[-1]
    angle = new_theta-theta
    for i in range (100):
        if action == 'drop':
                choose_grasp.assign()
        set_joint_positions(world.robot, world.base_joints, (x,y,theta + angle/100))
        time.sleep(0.01)
        theta += angle/100

    if action == 'open':
        pre_drawer_path = plan_arm(('pre_grip_drawer', [], []), world, algs=algs)
        executeArm(pre_drawer_path, world, action, tool_link, choose_grasp, gl, 'pre', id)

    if action == 'open':
        (base_x, base_y, theta) = get_joint_positions(world.robot, world.base_joints)
        (x, _, _) = get_joint_positions(world.kitchen, world.kitchen_joints)

        step = 100
        value = 0.3/step
        for s in range(step):
            base_x += value
            x += value
            set_joint_positions(world.robot, world.base_joints, (base_x, base_y, theta))
            set_joint_positions(world.kitchen, world.kitchen_joints, [0,x,0])
            time.sleep(0.01)
    else:
        arm_path = plan_arm(act, world, algs=algs)
        time.sleep(0.5)

        executeArm(arm_path, world, action, tool_link, choose_grasp, gl, 'act', id)
    time.sleep(0.5)
    post_path = plan_arm(('post_grip', [], []), world, algs=algs)
    time.sleep(0.5)

    executeArm(post_path, world, action, tool_link, choose_grasp, gl, 'post', id)

def executeArm(path, world, action, tool_link, choose_grasp, gl, motion, id):
    # for i in range(10):
    validFlag = True
    value = 0.3/len(path)
    prior = None
    for arm_pos in path:
        set_joint_positions(world.robot, world.arm_joints, arm_pos)
        arm_pose = get_link_pose(world.robot, tool_link)
        conf = next(closest_inverse_kinematics(world.robot, PANDA_INFO, tool_link, arm_pose, max_time=0.05), None)
        if conf is None:
            print('============================================Failure!=============================================')
            validFlag = False
            print(prior)
            wait_for_user()
            # time.sleep(0.01)
            breakpoint()
            conf = arm_pos
        if action == 'drop' and id != 0 and motion != 'post':
            choose_grasp.assign()
        if motion == 'post' and action == 'grip':
            choose_grasp.assign()
        if motion == 'act' and action == 'open':
            (_, x, _) = get_joint_positions(world.kitchen, world.kitchen_joints)
            set_joint_positions(world.kitchen, world.kitchen_joints, [0,x+value,0])
        set_joint_positions(world.robot, world.arm_joints, conf)
        prior = conf
    if action == 'grip' and len(gl) != 0 and motion != 'pre':
        choose_grasp.assign()
    
    if validFlag:
        print('ready to go')
        time.sleep(1)
        wait_for_user()



def execute_plan(plan):
    # print('Random seed:', get_random_seed())
    # print('Numpy seed:', get_numpy_seed())
    np.set_printoptions(precision=3, suppress=True)
    world = World(use_gui=True)
    sugar_box = add_sugar_box(world, idx=0, counter=1, pose2d=(-0.2, 0.65, np.pi / 4))
    spam_box = add_spam_box(world, idx=1, counter=0, pose2d=(0.2, 1.1, np.pi / 4))
    wait_for_user()
    world._update_initial()
    sample_fn = get_sample_fn(world.robot, world.arm_joints) 

    comparison_fn_rrt = (1.1400813978537498, 0.1787462256366006, -2.7093415878361893, -2.5805536889601135, -2.082868251419624, 0.6512893468496506,0.28984967118760885)
    
    conf = sample_fn()
    # set_joint_positions(world.robot, world.arm_joints, comparison_fn_rrt)
    set_joint_positions(world.robot, world.arm_joints, conf)

    wait_for_user()
        # sGrasp = Grasp(world, 4, SIDE_GRASP, 100, sugar_grip, pre_grip) #need to be pose
    # breakpoint()
    
    for i in plan:
        act = extract_act(i)
        execute_act(act, world, plan.index(i), algs='rrt_star')

    print("THE END")
    wait_for_user()
    world.destroy()


# if __name__ == '__main__':
#     plan = 
#     execute_plan()
