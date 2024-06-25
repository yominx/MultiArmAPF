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
NUM_ROBOTS = 3

JOINT_IDXS = arr([range(END_JOINT_IDX)])
ROBOT_IDXS = arr(range(NUM_ROBOTS))

ROBOT_PATH = "franka_panda/panda.urdf"
# ROBOT_PATH = "./3linkarm.urdf"

T_max = 400
SIM_TIMESTEP = 1
D_THRES   = 1
D_MIN     = 0.6
D_LOCK    = 0.3
CRIT_DIST = 0.2

###############################################################
################# Loading Robots Part #########################
###############################################################

# Load robot poses & task poses
startOri = [p.getQuaternionFromEuler([0,    0, 1*i*np.pi/2]) for i in range(NUM_ROBOTS)]
parOri   = [p.getQuaternionFromEuler([np.pi,0, 1*i*np.pi/2]) for i in range(NUM_ROBOTS)]
bases = arr([  [0,      0, 0], 
               [0.7,  0.7, 0],
               [1.2,    0, 0]  ])
assert(len(bases) == NUM_ROBOTS)

# Define tasks
 

traj1 = arr([  
               [ 0.6, 0.3, 0.4],
               [ 0.5,-0.3, 0.7],
               [ 0.6, 0.3, 0.2],
               [ 0.5,-0.3, 0.5],
                ])
traj2 = arr([  
               [ 0.4,-0.3, 0.5],
               [ 0.2,-0.5, 0.3],
               [ 0  ,-0.5, 0.6],
               [-0.3,-0.4, 0.7],
                ])
traj3 = arr([  
               [-0.6, 0.3, 0.8],
               [-0.8, 0.1, 0.6],
               [-0.6,-0.5, 0.6],
               [-0.4, 0.3, 0.9],
               ])

trajs = [traj1, traj2, traj3]
trajs = [ tr+bases[i] for i, tr in enumerate(trajs)]


# traj1 = list(map(lambda x: x+bases[0], traj1))
# traj2 = list(map(lambda x: x+bases[1], traj2))
# traj3 = list(map(lambda x: x+bases[1], traj2))
print(trajs)

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

import rrt_import
from robot_class import StateRobot

def end_all_tasks(robot_classes):
     for robot in robots_classes:
          if not robot.done:
               return False
     return True

def main():
     global T_max
     init_simulation()
     np.set_printoptions(precision=3)
     robots = []; robots_class = []; q_inits = []
     for idx in range(NUM_ROBOTS):
          robot = p.loadURDF(ROBOT_PATH, bases[idx], startOri[idx], useFixedBase=True)
          robots.append(robot)
          ori = p.getQuaternionFromEuler([np.pi,0,idx*np.pi])

     # To resolve IK
     rb = arr([[0, 0, 0, -np.pi/2, 0, np.pi/2, 0],[0, 0, 0, -np.pi/2, 0, np.pi/2, 0],[0, 0, 0, -np.pi/2, 0, np.pi/2, 0]])
     init_robots(robots, rb, start_index = 0)


     for idx, robot in enumerate(robots):
          q_inits.append(getNpIK(robot, END_JOINT_IDX, trajs[idx][0],parOri[idx]))
          wrapped = StateRobot(robot, robots, trajs[idx])
          robots_class.append(wrapped)

     robots = arr(robots); robots_class = arr(robots_class); q_inits = arr(q_inits)
     # input()

     init_robots(robots, q_inits, start_index = 0)
     # for joint in range(7):
          # p.resetJointState(robots[0], joint, q_inits[0][joint]) 
     p.stepSimulation()

     print(q_inits)
     # print(get_joint_angles(robots[0]))
     # print(get_joint_angles(robots[1]))

     # Draw Trajectories
     for ind, traj in enumerate(trajs):
          for i in range(len(traj)-1):
               p.addUserDebugLine(list(traj[i]), list(traj[i+1]), lineColorRGB=[.9, 1-0.25*ind, 0.25*ind], lineWidth=4)

     # for i in ROBOT_IDXS:
          # p.addUserDebugLine(list(pos_starts[i]), list(pos_ends[i]), lineColorRGB=[1,1,0], lineWidth=4)
          # drawLine(pos_starts[i], pos_ends[i], color=[1,1,0])
     
     position = []
     ## Start simulation
     lengths        = get_link_lengths(robots[0])
     crit_nums      = list(map(lambda x: int(np.ceil(x/CRIT_DIST)),lengths))
     crit_nums.append(1)

     robot_pairs = list(combinations(robots,2))
     two_joints_list = [list(range(1,END_JOINT_IDX+1)), list(range(1,END_JOINT_IDX+1))]
     joint_pairs = list(product(*two_joints_list))
     global timestep
     timestep = 0
     # input()
     time.sleep(1)
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
               for robot1,robot2 in robot_pairs:
                    for i,j in joint_pairs:
                         applyPotentialForce(robot1, robot2, i, j, crit_nums, robots_class)
                         dummyfn()
               # print(f"Time: {timestep}")
               timestep += 1; 
               if timestep % 100 == 0 and end_all_tasks(robot_classes): break
               p.stepSimulation()
               time.sleep(SIM_TIMESTEP * 2/240)

          except:
               raise
     input("\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\nEND")



def init_simulation():
     # Init sim
     physicsClient = p.connect(p.GUI)#or p.DIRECT for non-graphical version
     # Set up the visualizer
     p.configureDebugVisualizer(p.COV_ENABLE_WIREFRAME, 0)
     p.configureDebugVisualizer(p.COV_ENABLE_SHADOWS, 0)
     p.configureDebugVisualizer(p.COV_ENABLE_GUI, 1)
     p.setRealTimeSimulation(0)
     p.setTimeStep(1/60)
     # p.setTimeStep(SIM_TIMESTEP)
     # p.setGravity(0, 0, -9.8)
     p.resetDebugVisualizerCamera(cameraDistance=1, cameraYaw=15, cameraPitch=-20, cameraTargetPosition=[.6,0,0.5])




def simulationRobot(robot_id, joint_target):
     # p.setJointMotorControlArray(robot_id, [1,2], p.VELOCITY_CONTROL, forces=[0,0])
     # q_des = getNpIK(robot_id, END_JOINT_IDX, pos_target)
     # Set joint control!
     joints = getRevoluteJointList(robot_id)
     joint_target = [joint_target[q] for q in joints]
     Pscale = 0.05
     Dscale = 0.4
     Pvalue=arr([3, 3, 3, 2, 2, 2, 1])
     Dvalue=arr([3, 3, 3, 2, 2, 2, 1])
     p.setJointMotorControlArray(robot_id, joints, p.POSITION_CONTROL,
                                   targetPositions=joint_target,
                                   positionGains=Pscale*Pvalue, velocityGains=Dscale*Dvalue) # Tune the gains



def applyPotentialForce(robot_1, robot_2, joint_index1, joint_index2, crit_nums, robots_class): #it will be run inside the 2-layer-for-loop
     # print(joint_index1)
     # print(joint_index2)
     crits_list = [list(range(crit_nums[joint_index1-1])), list(range(crit_nums[joint_index2-1]))]
     crit_pairs = list(product(*crits_list))
     
     coeff = 1.2
     global D_MIN
     D_MIN = 0.2
     maxForce = coeff/(D_MIN**2)*(1/D_MIN - 1/D_THRES)


     base_pos = arr([getLinkPos(robot_1, joint_index1),   getLinkPos(robot_2, joint_index2)])
     link_vec = arr([getLinkPos(robot_1, min(joint_index1+1, END_JOINT_IDX)),
                     getLinkPos(robot_2, min(joint_index2+1, END_JOINT_IDX))]) - base_pos
     direc = arr([np.zeros_like(link_vec[0]) if norm(link_vec[0]) == 0 else link_vec[0]/norm(link_vec[0]),
                  np.zeros_like(link_vec[1]) if norm(link_vec[1]) == 0 else link_vec[1]/norm(link_vec[1])]) 

     for crit_1, crit_2 in crit_pairs:
          pos1 = base_pos[0] + CRIT_DIST * crit_1 * direc[0]
          pos2 = base_pos[1] + CRIT_DIST * crit_2 * direc[1]

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
          # p.addUserDebugPoints(pointPositions = [base_pos[0]],
                              # pointColorsRGB  = [[0,1,1]], pointSize = 50, lifeTime = 1)

          # if joint_index1 > 2:
          # global timestep
          # print(timestep)
          # if timestep%100==0 and robot_1 == 0:
               # drawLine(pos1, pos1 - 10*force*direction, color=[1, 0.2, 0.2],lifeTime=0.2,width=3)
          # if joint_index1 > 2 and robot_2 == 2:
               # drawLine(pos2, pos2 + 10*force*direction, color=[1, 0.2, 0.2],lifeTime=0.2,width=3)
     return 1






if __name__=="__main__":
     main()

