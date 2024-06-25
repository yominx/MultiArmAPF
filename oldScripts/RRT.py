import pybullet as p
import pybullet_data
import numpy as np
from numpy import array as arr
from numpy.linalg import norm 
from itertools import combinations, product

import random
from time import time

from .utils import INF, argmin, elapsed_time, BLUE, RED, apply_alpha

EPSILON = 1e-6
PRINT_FREQUENCY = 100

class RRTNode(object):

    def __init__(self, config, parent=None, d=0):
        self.config = config
        self.parent = parent
        self.children = set()
        self.d = d
        if parent is not None:
            self.cost = parent.cost + d
            self.parent.children.add(self)
        else:
            self.cost = d
        self.solution = False
        self.creation = iteration

    def set_solution(self, solution):
        if self.solution is solution:
            return
        self.solution = solution
        if self.parent is not None:
            self.parent.set_solution(solution)

    def retrace(self):
        if self.parent is None:
            return [self.config]
        return self.parent.retrace().append(self.config)

    def rewire(self, parent, d):
        if self.solution:
            self.parent.set_solution(False)
        self.parent.children.remove(self)
        self.parent = parent
        self.parent.children.add(self)
        if self.solution:
            self.parent.set_solution(True)
        self.d = d
        self.update()

    def update(self):
        self.cost = self.parent.cost + self.d
        for n in self.children:
            n.update()

    def clear(self):
        self.node_handle = None
        self.edge_handle = None


    def __str__(self):
        return self.__class__.__name__ + '(' + str(self.config) + ')'
    __repr__ = __str__



##################################################

def dist_joint(q1,q2):
	return norm(q2-q1)

def argmin(function, sequence):
    values = list(sequence)
    scores = list(map(function, sequence))
    index = numpy.argmin(scores)
    return values[index]

def safe_extend(q1,q2, collision_fn):
	RES = math.radian(2)
	diff = q2-q1
	q_new = q1+ RES * diff/norm(diff)

    if collision_fn(q_new):
    	return None
    return q_new


def rrt_star(robot, start, goal, sample_fn, collision_fn, radius=math.radian(21),
             max_time=INF, max_iterations=INF, goal_probability=.2):
    """
    :param start: Start configuration - conf
    :param goal: End configuration - conf
    :param distance_fn: Distance function - distance_fn(q1, q2)->float
    :param sample_fn: Sample function - sample_fn()->conf
    :param collision_fn: Collision function - collision_fn(q)->bool
    :param max_time: Maximum runtime - float
    :return: Path [q', ..., q"] or None if unable to find a solution
    """
    if collision_fn(start) or collision_fn(goal):
        return None
    nodes = [RRTNode(start)]
    goal_n = None
    start_time = time()
    iteration = 0
    while (elapsed_time(start_time) < max_time) and (iteration < max_iterations):
        do_goal = goal_n is None and (iteration == 0 or random() < goal_probability)
        sampled = goal if do_goal else sample_fn()
        # Informed RRT*
        if (goal_n is not None) and (dist_joint(start, sampled) + dist_joint(sampled, goal) >= goal_n.cost):
            continue
        iteration += 1

        nearest = argmin(lambda n: dist_joint(n.config, sampled), nodes)
        path = safe_extend(nearest.config, sampled, collision_fn)
        if path is None:
            continue
        new = RRTNode(path[-1], parent=nearest, d=dist_joint(
            nearest.config, path[-1]), path=path[:-1], iteration=iteration)
        # if safe and do_goal:
        if do_goal and (dist_joint(new.config, goal) < EPSILON):
            goal_n = new
            goal_n.set_solution(True)
        # TODO - k-nearest neighbor version
        neighbors = filter(lambda n: dist_joint(n.config, new.config) < radius, nodes)
        nodes.append(new)

        # TODO: smooth solution once found to improve the cost bound
        for n in neighbors:
            d = dist_joint(n.config, new.config)
            if (n.cost + d) < new.cost:
                path = safe_extend(n.config, new.config, collision_fn)
                if (path is not None) and (dist_joint(new.config, path[-1]) < EPSILON):
                    new.rewire(n, d, iteration=iteration)
            if (new.cost + d) < n.cost:
                path = safe_extend(new.config, n.config, collision_fn)
                if (path is not None) and (dist_joint(n.config, path[-1]) < EPSILON):
                    n.rewire(new, d, iteration=iteration)
    if goal_n is None:
        return None
    return goal_n.retrace()

