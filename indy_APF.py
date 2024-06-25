import pybullet as p
import pybullet_data
import time
import numpy as np
from numpy import array as arr
from numpy.linalg import norm 
from itertools import combinations, product
from p_utils import init_robots, get_joint_angles, getLinkPos, getNpIK, drawLine, dummyfn, getRevoluteJointList, set_joint_angles

from interfaces.device_socket_client import DeviceSocketClient
from interfaces.config_socket_client import ConfigSocketClient
from interfaces.control_socket_client import ControlSocketClient
from interfaces.rtde_socket_client import RTDESocketClient
import common as common

import rclpy
from rclpy.node import Node

from std_msgs.msg import String


###############################################################
################# Parameter Definition Part####################
###############################################################
END_JOINT_IDX = 6
ROBOT_PATH = "indy7.urdf"

D_THRES   = 0.6
D_MIN     = 0.3
D_LOCK    = 0.1
CRIT_DIST = 0.15
SELF_CRIT_DIST = 0.15



def get_link_lengths(robot):
     lengths = [] 
     prev = None
     for i in range(1,END_JOINT_IDX+1):
          curr = getLinkPos(robot, i)
          if prev is not None:
               dist = norm(curr - prev) 
               lengths.append(dist)
          prev = curr
     # print(lengths)
     return lengths


def end_all_tasks(robots_class, verbose = True):
     listDeadlock = []
     listDone = []
     result = True
     for robot in robots_class:
          listDeadlock.append(robot.deadlock)
          listDone.append(robot.done)
          if not robot.done:
               result = False
     if verbose:
          print(f"Deadlock Status: {listDeadlock}")
          print(f"Done Status:     {listDone}")
     return result

import rrt_import
from robot_class import StateRobot

def status_collision(robots):
     pair_candidate = list(combinations(robots,2))
     for robot1, robot2 in pair_candidate:
          pp = p.getContactPoints(robot1, robot2)
          if len(pp) > 0: return True

     return False

def makeRobotsNaive(robots, bases):
     naive_pose = arr([0, 0, 0, 0, 0, 0])
     rb = np.repeat(naive_pose, len(bases))
     rb = np.reshape(rb, (6,len(bases))).T
     init_robots(robots, rb, start_index = 0)

def makeTrajectoryJointSpace(robots, trajs, parOri):
     jointTrajs=[]
     for idx, robot in enumerate(robots):
          robot_joint_traj = []
          for wp_idx, waypoint in enumerate(trajs[idx]):
               orientation = None if wp_idx !=0 else parOri[idx]
               transformed = getNpIK(robot, END_JOINT_IDX, trajs[idx][wp_idx], orientation)
               robot_joint_traj.append(transformed)
          jointTrajs.append(robot_joint_traj)
     return jointTrajs

def drawWayPoints(robots, bases, jointTrajs):
     for ind, traj in enumerate(jointTrajs):
          p.addUserDebugText(f"Robot {ind}", bases[ind] + arr([0,0,1.5]), textColorRGB = [0,0,0], textSize = 1, lifeTime=15)
          prevpose = None
          robot = robots[ind]

          for i in range(len(traj)):
               set_joint_angles (robot, traj[i], endJointIndex=END_JOINT_IDX)
               pose = getLinkPos(robot, END_JOINT_IDX)
               if prevpose is not None:
                    p.addUserDebugLine(prevpose, pose, lineColorRGB=[.9, 1-0.12*ind, 0.25*ind], lineWidth=4)#, lifeTime = 10)
               prevpose = pose

def makeCriticalElements(robots,bases):
     lengths        = get_link_lengths(robots[0])
     crit_nums      = list(map(lambda x: int(np.ceil(x/CRIT_DIST)),lengths))
     crit_nums.append(1)
     self_crit_nums      = list(map(lambda x: int(np.ceil(x/SELF_CRIT_DIST)),lengths))
     self_crit_nums.append(1)

     pair_candidate = list(combinations(robots,2))
     robot_pairs = []
     for i, j in pair_candidate:
          if norm(bases[j] - bases[i]) < 2*0.85: #0.85 is workspace each
               robot_pairs.append((i,j))

     two_joints_list = [list(range(1,END_JOINT_IDX+1)), list(range(1,END_JOINT_IDX+1))]
     joint_pairs = list(product(*two_joints_list))
     return crit_nums, self_crit_nums, robot_pairs, joint_pairs

def testpart(trajs, bases, yaws, jointSpace = False, controller = None, rtde = None):
     startOri = [p.getQuaternionFromEuler([0,0,yaw]) for yaw in yaws]
     parOri   = [p.getQuaternionFromEuler([np.pi,0, yaw]) for yaw in yaws]
     robots = list(range(len(bases))); robots_class = []; q_inits = []; q_ends=[]
     for idx in range(len(bases)):
          p.resetBasePositionAndOrientation(idx, bases[idx], startOri[idx])
          # ori = p.getQuaternionFromEuler([np.pi,0,idx*np.pi])

     # To resolve IK
     makeRobotsNaive(robots, bases)

     jointTrajs = trajs if jointSpace is True else makeTrajectoryJointSpace(robots, trajs, parOri)
     for idx, robot in enumerate(robots):
          wrapped = StateRobot(robot, robots, jointTrajs[idx])
          robots_class.append(wrapped)

     robots = arr(robots); robots_class = arr(robots_class);
     q_inits = arr([jointTraj[0] for jointTraj in jointTrajs])

     # Check the task is feasible or not
     # for eachWayPoints in np.swapaxes(jointTrajs, 0, 1):
     #      init_robots(robots, eachWayPoints, start_index = 0)
     #      p.stepSimulation()
     #      input()
     #      if status_collision(robots): return -1

     init_robots(robots, q_inits, start_index = 0)
     p.stepSimulation()
     if status_collision(robots): return -1

     # Draw Trajectories
     print(get_link_lengths(robots[0]))
     drawWayPoints(robots, bases, jointTrajs)

     ## Start simulation
     crit_nums, self_crit_nums, robot_pairs, joint_pairs = makeCriticalElements(robots, bases)

     global timestep
     timestep = 0
     if rtde is not None:
          q_now = rtde.GetControlData()['q']
          q = arr(q_inits[0]) * 180 / np.pi
          controller.MoveJ(jstart=q_now , jtarget = q, blending_type=common.Property.BlendingType.OVERRIDE)
          input("Make sure that robot is stopped")

     while 1:
          try:
               for idx, robot in enumerate(robots):
                    tar = robots_class[idx].get_target(timestep)
                    if tar is None: continue
                    simulationRobot(robot, tar)

               # APF joint by joint
               for robot1,robot2 in robot_pairs:
                    for i,j in joint_pairs:
                         # applyPotentialForce(robot1, robot2, i, j, crit_nums, robots_class)
                         dummyfn()

               for idx, robot in enumerate(robots):
                    robot_class = robots_class[idx]
                    if robot_class.deadlock:
                         applySelfPF(robot_class, self_crit_nums)

               timestep += 1; 

               if controller is not None and timestep%10 == 0:                    
                    q_now = rtde.GetControlData()['q']
                    q = arr(get_joint_angles(robots[0])[:-1], dtype = np.float64) * 180 / np.pi
                    controller.MoveJ(jstart=q_now , jtarget = q, blending_type=common.Property.BlendingType.OVERRIDE)
                    # input()


               if timestep % 100 == 0 and end_all_tasks(robots_class):
                    return 1
               # if timestep == T_max: return 0
               p.stepSimulation()
               time.sleep(1/240)

          except:
               raise




def init_simulation():
     # Init sim
     physicsClient = p.connect(p.GUI, options = "--width=800 --height=800")#or p.DIRECT for non-graphical version
     # Set up the visualizer
     p.configureDebugVisualizer(p.COV_ENABLE_WIREFRAME, 0)
     p.configureDebugVisualizer(p.COV_ENABLE_SHADOWS, 0)
     p.configureDebugVisualizer(p.COV_ENABLE_GUI, 1)
     p.setRealTimeSimulation(0)
     p.setAdditionalSearchPath(pybullet_data.getDataPath())
     # p.setTimeStep(1/60)
     # p.setGravity(0, 0, -9.8)
     p.configureDebugVisualizer(p.COV_ENABLE_GUI,0)
     center = 0.4
     p.resetDebugVisualizerCamera(cameraDistance=1.5, cameraYaw=5, cameraPitch=-70, cameraTargetPosition=[center, -center, 0.5])


def simulationRobot(robot_id, joint_target):
     # p.setJointMotorControlArray(robot_id, [1,2], p.VELOCITY_CONTROL, forces=[0,0])
     # q_des = getNpIK(robot_id, END_JOINT_IDX, pos_target)
     # Set joint control!
     joints = getRevoluteJointList(robot_id)
     joint_target = [joint_target[q] for q in joints]
     Pscale = 0.01
     Dscale = 0.2
     Pvalue=arr([3, 3, 3, 2, 2, 2])
     Dvalue=arr([3, 3, 3, 2, 2, 2])
     p.setJointMotorControlArray(robot_id, joints, p.POSITION_CONTROL,
                                   targetPositions=joint_target,
                                   positionGains=Pscale*Pvalue, velocityGains=Dscale*Dvalue) # Tune the gains



def applyPotentialForce(robot_1, robot_2, joint_index1, joint_index2, crit_nums, robots_class): #it will be run inside the 2-layer-for-loop
     # print(joint_index1)
     crits_list = [list(range(crit_nums[joint_index1-1])), list(range(crit_nums[joint_index2-1]))]
     crits = (crit_nums[joint_index1-1], crit_nums[joint_index2-1])
     crit_pairs = list(product(*crits_list))
     coeff = 100
     global D_MIN
     D_MIN = 0.5
     maxForce = coeff/(D_MIN**2)*(1/D_MIN - 1/D_THRES)


     joint_pos = arr([getLinkPos(robot_1, joint_index1),   getLinkPos(robot_2, joint_index2)])
     link_vec = arr([getLinkPos(robot_1, min(joint_index1+1, END_JOINT_IDX)),
                     getLinkPos(robot_2, min(joint_index2+1, END_JOINT_IDX))]) - joint_pos
     # direc = arr([np.zeros_like(link_vec[0]) if norm(link_vec[0]) == 0 else link_vec[0]/norm(link_vec[0]),
                  # np.zeros_like(link_vec[1]) if norm(link_vec[1]) == 0 else link_vec[1]/norm(link_vec[1])]) 

     for crit_1, crit_2 in crit_pairs:
          # print(crit_1)
          pos1 = joint_pos[0] + (crit_1/crits[0]) * link_vec[0]
          pos2 = joint_pos[1] + (crit_2/crits[1]) * link_vec[1]
          dist = norm(pos2-pos1)
          # print(f'Distance is {dist}')
          if dist > D_THRES: continue
          if dist < D_LOCK: 
               return -1 # Deadlock occured
          if dist < D_MIN: 
               force = maxForce
          else: # D_MIN < d < D_THRES
               force = coeff/(dist**2)*(1/dist - 1/D_THRES)

          direction = (pos2-pos1)/dist # Direction unit vector

          if not robots_class[robot_2].deadlock and not robots_class[robot_2].done:
               p.applyExternalForce(robot_1, joint_index1, -force*direction, pos1, p.WORLD_FRAME)
               # drawLine(pos1, pos1 - 10*force*direction, color=[1, 0.2, 0.2],lifeTime=0.2,width=3)
          if not robots_class[robot_1].deadlock and not robots_class[robot_1].done:
               p.applyExternalForce(robot_2, joint_index2,  force*direction, pos2, p.WORLD_FRAME)
     return 1

def applySelfPF(robot_class, crit_nums):
     coeff = 30
     global D_MIN
     D_MIN = 0.3
     maxForce = coeff/(D_MIN**2)*(1/D_MIN - 1/D_THRES)

     robot = robot_class.robot
     joints = getRevoluteJointList(robot)
     self_joint_pairs = list(combinations(joints,2))

     joint_poses = [None for _ in range(END_JOINT_IDX+1)]
     for joint in joints:
          joint_poses[joint] = getLinkPos(robot, joint)

     for joint1, joint2 in self_joint_pairs:
          next_joint1 = min(joint1+1, END_JOINT_IDX)
          next_joint2 = min(joint2+1, END_JOINT_IDX)

          joint_pos = arr([joint_poses[joint1],      joint_poses[joint2]])
          link_vec = arr([joint_poses[next_joint1], joint_poses[next_joint2]]) - joint_pos
          # direc = arr([np.zeros_like(link_vec[0]) if norm(link_vec[0]) == 0 else link_vec[0]/norm(link_vec[0]),
          #              np.zeros_like(link_vec[1]) if norm(link_vec[1]) == 0 else link_vec[1]/norm(link_vec[1])]) 


          jointpair_crit_index = [list(range(crit_nums[joint1])), list(range(crit_nums[joint2]))]
          jointpair_crit_index = list(product(*jointpair_crit_index))
          crits = (crit_nums[joint1-1], crit_nums[joint2-1])
          
          for crit_1, crit_2 in jointpair_crit_index:
               pos1 = joint_pos[0] + (crit_1/crits[0]) * link_vec[0]
               pos2 = joint_pos[1] + (crit_2/crits[1]) * link_vec[1]
               # pos1 = base_pos[0] + CRIT_DIST * crit_1 * direc[0]
               # pos2 = base_pos[1] + CRIT_DIST * crit_2 * direc[1]

               dist = norm(pos2-pos1)
               # print(f'Distance is {dist}')
               if dist > D_THRES: continue
               if dist < D_LOCK: 
                    return -1 # Deadlock occured
               if dist < D_MIN: 
                    force = maxForce
               else: # D_MIN < d < D_THRES
                    force = coeff/(dist**2)*(1/dist - 1/D_THRES)

               direction = (pos2-pos1)/dist # Direction unit vector

               p.applyExternalForce(robot, joint1, -force*direction, pos1, p.WORLD_FRAME)
               p.applyExternalForce(robot, joint2,  force*direction, pos2, p.WORLD_FRAME)
               # drawLine(pos1, pos1 - 10*force*direction, color=[1, 0.2, 0.2],lifeTime=0.2,width=3)
     return



def randPi():
     return np.random.rand()*np.pi

def randXYZ(yaw = 0):
     limit_d = [0.3, 0.6]
     limit_z = [0.5,0.9]
     d = np.random.rand()*(limit_d[1] - limit_d[0]) + limit_d[0]
     theta = randPi() + yaw
     return arr([d*np.sin(theta), d*np.cos(theta), np.random.rand()*(limit_z[1]-limit_z[0])+limit_z[0]])

def trajGen(bases, yaws):
     trajs = []
     for base, yaw in zip(bases,yaws):
          print(type(randXYZ()))
          print(type(base))
          print(base)

          randStart = randXYZ(yaw) + base

          while True:
               randEnd = randXYZ(yaw) + base
               if norm(randEnd - randStart) > 0.3: break
          while True:
               randEnd2 = randXYZ(yaw) + base
               if norm(randEnd2 - randStart) > 0.3  and norm(randEnd2 - randEnd) > 0.3: break

          trajs.append([randStart, randEnd,randEnd2])
     return arr(trajs)

def initialize_robot_positions(num_robots, min_dist, radius):
     while True:
          r = radius * np.random.rand(num_robots,1); r = np.concatenate((r,r), axis = 1)
          theta = np.zeros_like(r)
          for i in range(num_robots):
               th = 2*randPi()
               theta[i] = [np.cos(th), np.sin(th)]

          # print(theta)
          bases = np.multiply(r, theta) 
          pair_candidate = list(combinations(bases,2))
          safe = True
          for i, j in pair_candidate:
               if norm(j-i) < min_dist: #too close!
                    safe = False
                    break
          if safe: 
               # print(bases)
               return bases

def init_indy(ip):
     # ip = '192.168.151.248'
     controller = ControlSocketClient(ip)
     # device = DeviceSocketClient(ip)
     # config = ConfigSocketClient(ip)
     rtde = RTDESocketClient(ip)

     # control.GetVariableNameList()
     # control.GetIntVariable()
     return controller, rtde


def main():
     NUM_ROBOTS = 2

     startOri = [p.getQuaternionFromEuler([0, 0, 0]) for i in range(NUM_ROBOTS)]
     init_simulation()
     for idx in range(NUM_ROBOTS):
          robot = p.loadURDF(ROBOT_PATH, [idx, 0, 0], startOri[idx], useFixedBase=True)
     p.loadURDF("plane.urdf",[0, 0, 0])

     np.set_printoptions(precision=3)

     yaws = [0 for i in range(NUM_ROBOTS)]
     bases = arr([[0,0], [0.757,0]])
     zeros = np.zeros((NUM_ROBOTS,1))
     bases = np.concatenate((bases, zeros), axis=1)
     # trajs = trajGen(bases, yaws)
     trajs =  arr([
               [
               [0,0,-90,0,-90,0],
               [0,0,0,0,0,0],
               [20,20,-90,20,-90,0],
               [0,30,0,30,0,0],
               [0,0,-90,0,-90,0],
               [0,20,0,20,0,0],
               ],
               [
               [0,0,-90,0,-90,0],
               [0,0,0,0,0,0],
               [20,20,-90,20,-90,0],
               [0,30,0,30,0,0],
               [0,0,-90,0,-90,0],
               [0,20,0,20,0,0],
               ]
               ],dtype=np.float64)
     trajs *=  np.pi / 180 # Degree to radian
     ip = '192.168.151.248'
     controller,rtdata = None, None
     # controller, rtdata = init_indy(ip)
     # q_now = rtdata.GetControlData()['q']
     # controller.MoveJ(jstart=q_now , jtarget = [0,0,0,0,0], blending_type=common.Property.BlendingType.OVERRIDE,vel_ratio = 50,acc_ratio = 50)

     success = testpart(trajs, bases, yaws, jointSpace = True, controller = controller, rtde = rtdata)


if __name__=="__main__":
     main()

