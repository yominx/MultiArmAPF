import pybullet as p
import numpy as np
from numpy import array as arr
from numpy.linalg import norm 
from itertools import combinations, product
import importlib
import random
from time import time

from pybulletplanning.pybullet_tools.utils import plan_joint_motion
from p_utils import getLinkPos, getNpIK, drawLine, set_joint_angles, getRevoluteJointList
    
EPSILON = 1e-6
PRINT_FREQUENCY = 100

def get_total_length(robot):
     total_length = 0.
     prev = None
     jointNum = p.getNumJoints(robot)
     for i in range(1,jointNum):
          curr = getLinkPos(robot, i)
          if prev is not None:
               dist = norm(curr - prev) 
               total_length += dist
          prev = curr
     return total_length


def get_nearby_robots(robot, robot_list, reachable_length=None):
    if reachable_length is None: reachable_length = get_total_length(robot)
    def get_base_pos(body):
        return arr(p.getLinkState(body, 0)[0])
    
    base_pos = get_base_pos(robot)
    near_list = []
    for target_robot in robot_list:
        tar_pos = get_base_pos(target_robot)
        distance = norm(tar_pos - base_pos)
        if distance == 0 or distance > 2 * reachable_length:
            continue
        near_list.append(target_robot)
    return near_list


def plan(robot, robots, target_pose, movable_joints = None, algorithm = None, MAX_TRIAL = 10, useTaskSpace = True):
    if movable_joints is None:
        movable_joints = getRevoluteJointList(robot) #list(range(1,p.getNumJoints(robot)-1)) # skip base joint and EE joint
    # for i in range(MAX_TRIAL):
    if useTaskSpace:
        tar_joint_state = getNpIK(robot, max(movable_joints)+1, target_pose)
    else:
        tar_joint_state = target_pose
    print(movable_joints)
    print(tar_joint_state)
    near_obs = get_nearby_robots(robot, robots)
    # tar_joint_state = [tar_joint_state[idx] for idx in movable_joints]
    # print(tar_joint_state)


    # for joint in movable_joints:
    #     p.resetJointState(robot, joint, tar_joint_state[joint]) 
    # input()
    path = plan_joint_motion(body=robot, joints=movable_joints, end_conf=tar_joint_state,
                        obstacles = near_obs, max_distance = 0.1, cache = False, algorithm = algorithm, self_collisions=False, max_time=2)
    # if path is not None:
    #     return path
    return path
    # return None

def show_path(robot, path, wait = True):
    jointNum = p.getNumJoints(robot)
    for joint_tar in path:
        for i in range(1,jointNum-1):
            p.resetJointState(robot, i, joint_tar[i-1], targetVelocity=0)

        end_pose = getLinkPos(robot, jointNum-1)
        p.addUserDebugPoints(pointPositions = [end_pose], pointColorsRGB=[[.8,.2,.2]], pointSize=5)

    for i in range(1,jointNum-1):
        p.resetJointState(robot, i, path[0][i-1], targetVelocity=0)
    if wait:
        input("Waiting")
    return