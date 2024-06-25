import pybullet as p
import pybullet_data
import time
import numpy as np
from numpy import array as arr
from numpy.linalg import norm 
from itertools import combinations, product


from p_utils import init_robots, get_joint_angles, getLinkPos, getNpIK, drawLine, dummyfn, getRevoluteJointList

###############################################################
################# Parameter Definition Part####################
###############################################################
END_JOINT_IDX = 7
ROBOT_PATH = "franka_panda/panda_nohand.urdf"
# ROBOT_PATH = "franka_panda/panda.urdf"
# ROBOT_PATH = "ur5/ur5.urdf"

SIM_TIMESTEP = 1
D_THRES   = 0.5
D_MIN     = 0.3
D_LOCK    = 0.1
CRIT_DIST = 0.15
SELF_CRIT_DIST = 0.15

###############################################################
################# Loading Robots Part #########################
###############################################################

# Load robot poses & task poses


###############################################################


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
     naive_pose = arr([0, 0, 0, -np.pi/2, 0, np.pi/2, 0])
     rb = np.repeat(naive_pose, len(bases))
     rb = np.reshape(rb, (7,len(bases))).T
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


def testpart(trajs, bases, yaws, T_max):
     startOri = [p.getQuaternionFromEuler([0,0,yaw]) for yaw in yaws]
     parOri   = [p.getQuaternionFromEuler([np.pi,0, yaw]) for yaw in yaws]

     print("asd")

     robots = list(range(len(bases))); robots_class = []; q_inits = []; q_ends=[]
     for idx in range(len(bases)):
          p.resetBasePositionAndOrientation(idx, bases[idx], startOri[idx])
          # ori = p.getQuaternionFromEuler([np.pi,0,idx*np.pi])

     # To resolve IK
     makeRobotsNaive(robots, bases)

     jointTrajs = makeTrajectoryJointSpace(robots, trajs, parOri)
     for idx, robot in enumerate(robots):
          # q_inits.append(getNpIK(robot, END_JOINT_IDX, trajs[idx][0], parOri[idx]))
          # q_ends.append(getNpIK(robot, END_JOINT_IDX, trajs[idx][-1]))
          # wrapped = StateRobot(robot, robots, trajs[idx])
          wrapped = StateRobot(robot, robots, jointTrajs[idx])
          robots_class.append(wrapped)

     robots = arr(robots); robots_class = arr(robots_class); #q_inits = arr(q_inits); q_ends = arr(q_ends)
     q_inits = arr([jointTraj[0] for jointTraj in jointTrajs])
     q_ends = arr([jointTraj[-1] for jointTraj in jointTrajs])

     print("check end conf")

     # Check the task is feasible or not
     init_robots(robots, q_ends, start_index = 0)
     p.stepSimulation()

     if status_collision(robots): return -1, None
     print("check init conf")

     init_robots(robots, q_inits, start_index = 0)
     # print(arr(jointTrajs).T)
     # for eachWayPoints in np.swapaxes(jointTrajs, 0, 1):
     #      init_robots(robots, eachWayPoints, start_index = 0)
     #      p.stepSimulation()
     #      if status_collision(robots): return -1, None
     for ind, traj in enumerate(trajs):
          # p.addUserDebugText(f"Robot {ind}", bases[ind] + arr([0,0,1.5]), textColorRGB = [0,0,0], textSize = 1, lifeTime=10)
          p.addUserDebugPoints(pointPositions = traj, pointColorsRGB  = [[9,.3,.3],[.9,.9,0],[.4,.7,.4]],
                               pointSize = 20, replaceItemUniqueId=rays[ind])
         # Draw Trajectories
          for i in range(len(traj)-1):
               p.addUserDebugLine(list(traj[i]), list(traj[i+1]), lineColorRGB=[.9, 1-0.2*ind, 0.3*ind], lineWidth=3,replaceItemUniqueId=rays2[2*ind+i])

     print("asd")
     
     position = []
     ## Start simulation
     lengths        = get_link_lengths(robots[0])
     crit_nums      = list(map(lambda x: int(np.ceil(x/CRIT_DIST)),lengths))
     crit_nums.append(1)
     self_crit_nums      = list(map(lambda x: int(np.ceil(x/SELF_CRIT_DIST)),lengths))
     self_crit_nums.append(1)

     # if status_collision(robots): return -1
     pair_candidate = list(combinations(robots,2))
     robot_pairs = []
     for i, j in pair_candidate:
          if norm(bases[j] - bases[i]) < 2*0.85: #0.85 is workspace each
               robot_pairs.append((i,j))

     two_joints_list = [list(range(1,END_JOINT_IDX+1)), list(range(1,END_JOINT_IDX+1))]
     joint_pairs = list(product(*two_joints_list))
     global timestep
     timestep = 0


     # print(q_inits)
     # p.configureDebugVisualizer(p.COV_ENABLE_RENDERING, 1)
     for _ in range(20):
          for ind, q_ in enumerate(q_inits):
               simulationRobot(ind, q_)
               # p.stepSimulation()
               # time.sleep(0.1)

     if status_collision(robots): return -1, None

     # time.sleep(1)
     timeinfo = []
     while 1:
          try:
               # PD position control
               # print(f"{getLinkPos(2, 4)}")
               for idx, robot in enumerate(robots):
                    # if idx == 1: continue
                    tar = robots_class[idx].get_target(timestep)
                    # if idx == 1 and timestep%300 == 0: 
                         # print("target:")
                         # print(tar)
                         # print(get_joint_angles(robot))
                    if tar is None: continue
                    simulationRobot(robot, tar)

               # APF joint by joint
               starttime = time.time()
               for robot1,robot2 in robot_pairs:
                    for i,j in joint_pairs:
                         # print()
                         applyPotentialForce(robot1, robot2, i, j, crit_nums, robots_class)
               for idx, robot in enumerate(robots):
                    robot_class = robots_class[idx]
                    if robot_class.deadlock or robot_class.done:
                         # print(robot_class.deadlock)
                         applySelfPF(robot, robot_class, self_crit_nums[:-1])

               # print(f"Time: {timestep}")
               timestep += 1; 
               if end_all_tasks(robots_class, verbose=False):
                    return 1, timeinfo
               if timestep == T_max: 
                    return 0, None
               if status_collision(robots):
                    if timestep < 20: 
                         return -1, None 
                    else:
                         # input()
                         return 2, None

               # p.configureDebugVisualizer(p.COV_ENABLE_RENDERING, 1)
               p.stepSimulation()
               timediff = time.time() - starttime
               timeinfo.append(timediff)
               time.sleep(SIM_TIMESTEP * 4/240)
               # p.configureDebugVisualizer(p.COV_ENABLE_RENDERING, 0)

          except:
               raise




def init_simulation():
     # Init sim
     physicsClient = p.connect(p.GUI, options = "--width=1400 --height=1200")#or p.DIRECT for non-graphical version
     # physicsClient = p.connect(p.DIRECT, options = "--width=1400 --height=1200")#or p.DIRECT for non-graphical version
     # Set up the visualizer
     p.configureDebugVisualizer(p.COV_ENABLE_WIREFRAME, 0)
     p.configureDebugVisualizer(p.COV_ENABLE_SHADOWS, 0)
     # p.configureDebugVisualizer(p.COV_ENABLE_GUI, 1)
     p.setRealTimeSimulation(0)
     p.setAdditionalSearchPath(pybullet_data.getDataPath())
     # p.setTimeStep(1/60)
     # p.setTimeStep(SIM_TIMESTEP)
     # p.setGravity(0, 0, -9.8)
     p.configureDebugVisualizer(p.COV_ENABLE_GUI,0)
     center = 0
     p.resetDebugVisualizerCamera(cameraDistance=1.5, cameraYaw=5, cameraPitch=-70, cameraTargetPosition=[center, center, 0.5])


def simulationRobot(robot_id, joint_target):
     # p.setJointMotorControlArray(robot_id, [1,2], p.VELOCITY_CONTROL, forces=[0,0])
     # q_des = getNpIK(robot_id, END_JOINT_IDX, pos_target)
     # Set joint control!
     joints = getRevoluteJointList(robot_id)
     # joint_target = [joint_target[q] for q in joints]
     Pscale = 0.12  
     Dscale = 0.3
     Pvalue=arr([3, 3, 3, 2, 2, 2, 1])
     Dvalue=arr([3, 3, 3, 2, 2, 2, 1])
     # Pvalue=arr([3, 3, 3, 2, 2, 2])
     # Dvalue=arr([3, 3, 3, 2, 2, 2])
     p.setJointMotorControlArray(robot_id, joints, p.POSITION_CONTROL,
                                   targetPositions=joint_target,
                                   positionGains=Pscale*Pvalue, velocityGains=Dscale*Dvalue) # Tune the gains



def applyPotentialForce(robot_1, robot_2, joint_index1, joint_index2, crit_nums, robots_class): #it will be run inside the 2-layer-for-loop
     # print(joint_index1)
     crits_list = [list(range(crit_nums[joint_index1-1])), list(range(crit_nums[joint_index2-1]))]
     crits = (crit_nums[joint_index1-1], crit_nums[joint_index2-1])
     crit_pairs = list(product(*crits_list))
     coeff = 70
     global D_MIN
     D_MIN = 0.2
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
          # pos1 = joint_pos[0] + CRIT_DIST * crit_1 * direc[0]
          # pos2 = joint_pos[1] + CRIT_DIST * crit_2 * direc[1]

          # p.addUserDebugPoints(pointPositions = [pos1],
          #                     pointColorsRGB  = [[0,1,1]], pointSize = 50, lifeTime = 1)
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
          if not robots_class[robot_1].deadlock and not robots_class[robot_1].done:
               p.applyExternalForce(robot_2, joint_index2,  force*direction, pos2, p.WORLD_FRAME)
          # p.addUserDebugPoints(pointPositions = [pos1],
          #                     pointColorsRGB  = [[0,1,1]], pointSize = 50, lifeTime = 1)

          # if joint_index1 > 2:
          # global timestep
          # print(timestep)
          # if timestep%100==0:
               # drawLine(pos1, pos1 - 10*force*direction, color=[1, 0.2, 0.2],lifeTime=0.2,width=3)
          # if joint_index1 > 2 and robot_2 == 2:
               # drawLine(pos2, pos2 + 10*force*direction, color=[1, 0.2, 0.2],lifeTime=0.2,width=3)
     return 1

def applySelfPF(robot, robot_class, crit_nums):
     coeff = 50
     global D_MIN
     D_MIN = 0.2
     maxForce = coeff/(D_MIN**2)*(1/D_MIN - 1/D_THRES)


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
     limit_d = [0.4, 0.7]
     limit_z = [0.55,0.8]
     d = np.random.rand()*(limit_d[1] - limit_d[0]) + limit_d[0]
     theta = randPi() + yaw
     return [d*np.sin(theta), d*np.cos(theta), np.random.rand()*(limit_z[1]-limit_z[0])+limit_z[0]]

def trajGen(bases, yaws):
     trajs = []
     for base, yaw in zip(bases,yaws):
          # print(randXYZ())
          randStart = randXYZ(yaw) + base
          while True:
               randEnd = randXYZ(yaw) + base
               if 0.3 < norm(randEnd - randStart) < 0.7: break
          while True:
               randEnd2 = randXYZ(yaw) + base
               if 0.3 < norm(randEnd2 - randStart) < 0.7  and 0.3 < norm(randEnd2 - randEnd) < 0.7: break

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

import sys
def main():
     NUM_ROBOTS = int(sys.argv[1])

     RADIUS = .7*np.sqrt(NUM_ROBOTS)
     MIN_DIST_ROBOT = .8
     T_max = 2400
     total_iteration = 200
     WRITE = True

     startOri = [p.getQuaternionFromEuler([0, 0, 0]) for i in range(NUM_ROBOTS)]
     init_simulation()
     for idx in range(NUM_ROBOTS):
          robot = p.loadURDF(ROBOT_PATH, [idx, 0, 0], startOri[idx], useFixedBase=True)
     p.loadURDF("plane.urdf",[0, 0, 0])

     np.set_printoptions(precision=3)

     if WRITE:
          f = open(f'./data_final/RRT{NUM_ROBOTS}.txt', 'w')
          f.write(f'mean  std  running_time\n')
     global rays, rays2
     rays= []
     rays2= []
     for i in range(3*NUM_ROBOTS):
          rays.append(p.addUserDebugPoints(pointPositions = [[0,0,0],[0,0,0],[0,0,0]], pointColorsRGB  = [[1,.2,.2],[1,.2,.2],[1,.2,.2]], pointSize = 1))
     for i in range(2*NUM_ROBOTS):
          rays2.append(p.addUserDebugLine([0,0,0], [0,0,0], [1,1,1]))

     success_iteration = 0; curr_iteration = 0;
     while curr_iteration < total_iteration:
          yaws = [2*randPi() for i in range(NUM_ROBOTS)]
          # yaws = [0.5*i*np.pi for i in range(NUM_ROBOTS)]
          print("Generating safe bases...")
          bases = initialize_robot_positions(NUM_ROBOTS, MIN_DIST_ROBOT, RADIUS)
          print("Done.")

          zeros = np.zeros((NUM_ROBOTS,1))
          bases = np.concatenate((bases, zeros), axis=1)
          print("GEN!!!")
          trajs = trajGen(bases, yaws)
          # print(trajs)
          # print(bases)
          print("Start test...")
          success, timeinfo = testpart(trajs, bases, yaws, T_max)
          if success == -1:
               # print("Infeasible task... Regenerating.")
               continue

          if success == 0 and WRITE:
               f.write(f'0  0  0\n')

          if success == 2 and WRITE:
               f.write(f'-1  -1  -1\n')

          if success == 1 and WRITE:
               success_iteration += 1 
               info = arr(timeinfo)
               std = np.std(info); mean = np.mean(info); running_time = len(info) /240.
               f.write(f'{mean}  {std}  {running_time}\n')


          curr_iteration+=1
          print(f"{curr_iteration:} Success rate is {success_iteration / curr_iteration * 100}%")

     print(f"Success rate is {success_iteration / total_iteration * 100}%")



def main2():
     NUM_ROBOTS = 2
     RADIUS = .7*np.sqrt(NUM_ROBOTS)
     MIN_DIST_ROBOT = .8
     T_max = 2400
     WRITE = True

     startOri = [p.getQuaternionFromEuler([0, 0, 0]) for i in range(NUM_ROBOTS)]
     init_simulation()
     for idx in range(NUM_ROBOTS):
          robot = p.loadURDF(ROBOT_PATH, [idx, 0, 0], startOri[idx], useFixedBase=True)
     # p.loadURDF("plane.urdf",[0, 0, 0])

     np.set_printoptions(precision=3)

     global rays, rays2
     rays= []
     rays2= []
     # for i in range(3*NUM_ROBOTS):
          # rays.append(p.addUserDebugPoints(pointPositions = [[0,0,0],[0,0,0],[0,0,0]], pointColorsRGB  = [[1,.2,.2],[1,.2,.2],[1,.2,.2]], pointSize = 1))
     # for i in range(2*NUM_ROBOTS):
          # rays2.append(p.addUserDebugLine([0,0,0], [0,0,0], [1,1,1]))

     success_iteration = 0; curr_iteration = 0;
     # yaws = [2*randPi() for i in range(NUM_ROBOTS)]
     yaws = [i*np.pi for i in range(NUM_ROBOTS)]
     bases = arr([
               [0,0,0],
               [.8,0,0],
          ])

     print("GEN!!!")
     trajs = arr([[
               # bases[0]+[0,0,1],
               [0.4,0.4,0.5],
               [0.4,-0.6,0.5],
               # [0.4,.3,.5],
               [0.4,-0.6,0.8],
               # bases[0]+[0.2,0,1],
          ],[
               # bases[1]+[0,0,1],
               [0.4,-0.4,0.5],
               [0.4,0.6,0.5],
               [0.4,0.6,0.8],
               # bases[1]+[-0.2,0,1],
          ]],dtype=np.float64)
     print("Start test...")
     for i in range(len(trajs[0])*NUM_ROBOTS):
          rays.append(p.addUserDebugPoints(pointPositions = [[0,0,0],[0,0,0],[0,0,0]], pointColorsRGB  = [[1,.2,.2],[1,.2,.2],[1,.2,.2]], pointSize = 1))
     for i in range((len(trajs[0])-1)*NUM_ROBOTS):
          rays2.append(p.addUserDebugLine([0,0,0], [0,0,0], [1,1,1]))

     success, timeinfo = testpart(trajs, bases, yaws, T_max)

if __name__=="__main__":
     # main()
     main2()

