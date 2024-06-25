import pybullet as p
import pybullet_data
import time
import numpy as np
from numpy import array as arr
from numpy.linalg import norm 
from itertools import combinations, product
DIST_THRESHOLD = 0.03

def init_robots(robots, q_inits, start_index = 1):
     # jointNum = p.getNumJoints(robots[0])
     # print(jointNum)
     jointNum = len(q_inits[0])
     # print(jointNum)
     for rb_idx, robot in enumerate(robots):
          for joint_i in range(jointNum):
               p.resetJointState(robot, start_index + joint_i, q_inits[rb_idx][joint_i], targetVelocity=0.0)

def getJointList(robot):
     return list(range(p.getNumJoints(robot)))

def getRevoluteJointList(robot):
     jointList = getJointList(robot)
     revList = [joint for joint in jointList if (p.getJointInfo(robot, joint)[2] == p.JOINT_REVOLUTE)] # jointInfo[2] is jointType
     return revList

def get_joint_angles(robot):
     jointList = getJointList(robot)
     return [p.getJointState(robot, joint)[0] for joint in jointList]

def set_joint_angles(robot, angles, endJointIndex = None):
     jointList = getJointList(robot)
     if endJointIndex is None:
          assert len(jointList) == len(angles)
     return [p.resetJointState(robot, jointList[ind], angle) for ind, angle in enumerate(angles)]

def getLinkPos(robot_id, joint_index):
     return arr(p.getLinkState(robot_id, joint_index)[0])
     
def getNpIK(robot_id, joint_index, position, targetOrientation=None):
     if targetOrientation is None:
          return arr(p.calculateInverseKinematics(robot_id, joint_index, position, maxNumIterations=100))
     return arr(p.calculateInverseKinematics(robot_id, joint_index, position, targetOrientation=targetOrientation, maxNumIterations=100))


def drawLine(pos_start, pos_end, color,lifeTime=100,width = 3):
     p.addUserDebugLine(pos_start, pos_end, lineColorRGB=color, lifeTime=lifeTime, lineWidth=width)

def dummyfn():
     return