import pybullet as p
import numpy as np
from numpy import array as arr
from numpy.linalg import norm 
from itertools import combinations, product
import importlib
import random
from time import time

from pybulletplanning.pybullet_tools.utils import plan_joint_motion
from p_utils import getRevoluteJointList, getLinkPos, getNpIK, drawLine, get_joint_angles, set_joint_angles
import rrt_import

def ang_diff(ang1, ang2):
    diffs = []
    for a1, a2 in zip(ang1, ang2):
        diff = (a2 - a1 + 2 * np.pi) % 2*np.pi
        if diff > np.pi:
            diff = 2*np.pi - diff
        diffs.append(diff)
    return arr(diffs)



class StateRobot(object):

    def __init__(self, robot, robots = [], trajectory=[]):
        self.robot = robot
        self.goal_pose = None
        self.traj = trajectory
        self.iter_path = iter(trajectory)
        self.last_timestep = 0
        self.NUM_JOINT = p.getNumJoints(robot)
        self.THR_CONTROL = .07
        self.THR_DONE = .2
        self.THR_DEADLOCK = .4
        self.THR_UNLOCK_VEL = 0.6
        self.THR_TIMESTEP = 10
        # self.THR_TIMESTEP = 10
        self._reset_state()
        self.near_robots = rrt_import.get_nearby_robots(robot, robots)
        self.revJoints = getRevoluteJointList(self.robot)

        next(self.iter_path) # pass initial condition
        self.goal_pose = self.get_next_goal()


    def _reset_state(self):
        self.path = None
        self.tar_pose = None
        self.done = False
        self.index = 0
        self.control = True
        self.deadlock = False

    def _update(self):
        # print(self.path)
        if self.path is None or self.goal_pose is None or self.done:
            self.control = False
            return
        self._check_deadlock()
        if self.deadlock or self.path is None: 
            # self.path = None
            self.control = False
            return
        self.control = True

    def _check_deadlock(self):
        if not self.deadlock:
            cur_state = get_joint_angles(self.robot)
            cur_state = [cur_state[q] for q in self.revJoints]

            if norm(arr(self.tar_pose) - arr(cur_state)) > self.THR_DEADLOCK:
                self.deadlock = True
                print(f"!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!")
                print(f"!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!")
                print(f"!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!")
                print(f"!!!!!!!! Robot {self.robot} fall in DEADLOCK!!!!!!!!")
                print(f"!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!")
                print(f"!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!")
                print(f"!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!")
        
        elif self.deadlock:
            cur_state = get_joint_angles(self.robot)
            cur_state = [cur_state[q] for q in self.revJoints]
            velocities = [p.getJointState(self.robot, joint)[1] for joint in self.revJoints]
            speed = norm(velocities)
            pos_error = norm(arr(self.path[self.index]) - arr(cur_state))

            if speed < self.THR_UNLOCK_VEL:# and pos_error < self.THR_CONTROL:
                self._unlock_replan()

    def _unlock_replan(self):
        # it requires unlock deadlock
        print(f"TRY: Move to {self.goal_pose}")
        self.deadlock = False
        if self.plan(self.goal_pose) == 1:
            print(f"!!!UNLOCK {self.robot}!!!!!!!!")
            print(f"Current joint state: {[p.getJointState(self.robot, joint)[0] for joint in self.revJoints]}")
            self._reset_state()


    def plan(self, target_position, robots = None, draw_path = False):
        init_joint_angles = get_joint_angles(self.robot)
        # print(init_joint_angles[:7])
        if robots is None:
            robots = self.near_robots
        self.goal_pose = target_position
        # self.revJoints = getRevoluteJointList(self.robot)
        path = rrt_import.plan(self.robot, robots, self.goal_pose, movable_joints=self.revJoints, algorithm = None, useTaskSpace=False)
        self._reset_state()
        self.path = path
        
        if draw_path and path is not None:
            print("Drawing path")
            rrt_import.show_path(self.robot, path, wait = False)
        set_joint_angles(self.robot, init_joint_angles)
        if path is None:
            # print("Failed to generate trajectory..")
            return 
        # print("Path is generated Successfully")
        self.tar_pose = path[0]





    def get_target(self, timestep = 0):
        # if self.robot == 2:
            # print(self.goal_pose)
        if self.path is None and not self.done:
            if timestep - self.last_timestep >= self.THR_TIMESTEP:
                self.plan(self.goal_pose)
                if self.path is None:
                    self.last_timestep = timestep


        self._update()
        cur_state = get_joint_angles(self.robot)
        cur_state = [cur_state[q] for q in self.revJoints]
        if self.deadlock:
            # cur_state = get_joint_angles(self.robot)
            # cur_state = [cur_state[q] for q in self.revJoints]
            return cur_state
        if self.done:
            return cur_state
            # return self.path[self.index]
            # return arr([0, 0, 0, -0.01, 0, np.pi/2, 0])
        if not self.control:
            return None
        self.tar_pose = self.path[self.index]
        # Check close enough

        pos_error = norm(arr(self.tar_pose)[:-1] - arr(cur_state)[:-1])
        time_error = timestep - self.last_timestep


        # if self.robot == 0 :
        #     print(self.tar_pose)
        #     print(cur_state)
        #     print(f"ERROR: {pos_error}")


        if pos_error < self.THR_CONTROL or time_error > self.THR_TIMESTEP:
            self.last_timestep = timestep

            if self.index+1 < len(self.path): 
                self.index = self.index + 1
                self.tar_pose = self.path[self.index]

            elif pos_error > self.THR_DONE:
                self.tar_pose = self.path[self.index]

            else: # End of the planned trajectory
                # done_error = norm(arr(self.goal_pose) - arr(get_joint_angles(self.robot)[1:-1]))
                # done_error = norm(ang_diff(self.goal_pose, arr(get_joint_angles(self.robot)[1:-1])))
                # if self.robot == 0 :
                    # print("_________")
                    # print(f"ERROR: {done_error}")
                    # print(f"{self.goal_pose}")
                    # print(f"{get_joint_angles(self.robot)[1:-1]}")
                if pos_error < self.THR_DONE:
                    self.goal_pose = self.get_next_goal()
                    if self.goal_pose is None:
                        self.done = True
                        # print(f"!!!!!!!!!!!!!!!!!!!!!!!!!!")
                        print(f"ROBOT {self.robot} DONE")
                        # print(f"!!!!!!!!!!!!!!!!!!!!!!!!!!")
                        # return None
                        return cur_state#get_joint_angles(self.robot)
                    self.plan(self.goal_pose)
                    self.tar_pose = cur_state#get_joint_angles(self.robot)

        return self.tar_pose



    def get_next_goal(self):
        try:
            # print(f"Robot {self.robot + 1} reached waypoint... try to gather next waypoint")
            result = next(self.iter_path)
            return result
        except StopIteration:
            return None
