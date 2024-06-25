import pybullet as p
import pybullet_data
import time
import numpy as np
from numpy import array as arr
from numpy.linalg import norm 
from itertools import combinations, product

from robot_class import StateRobot
from p_utils import init_robots, getLinkPos, getNpIK, drawLine, dummyfn

###############################################################
################# Parameter Definition Part####################
###############################################################
END_JOINT_IDX = 3
NUM_ROBOTS = 2

JOINT_IDXS = arr([range(END_JOINT_IDX)])
ROBOT_IDXS = arr(range(NUM_ROBOTS))

ROBOT_PATH = "./3linkarm.urdf"

T_max = 400
SIM_TIMESTEP = 1
D_THRES   = 0.8
D_MIN     = 0.6
D_LOCK    = 0.3
CRIT_DIST = 0.2

###############################################################
################# Loading Robots Part #########################
###############################################################

# Load robot poses & task poses
startOri = p.getQuaternionFromEuler([0,0,0])
bases = arr([  [0,   0, 0], 
               [2.5, -0.5, 0]  ])
assert(len(bases) == NUM_ROBOTS)

# Define tasks

pos_starts=arr([ [ 2.1, 0.9, 0.1],
                 [-1, .5, 0.1] ])
                 # [-0.4,-0.5, 0.1] ])
pos_ends  =arr([ [ -0.0,-1.0, 0.1],
                 [-1.8, 1.5, 0.1] ])
# pos_starts=arr([ [ 1.8, 1.2, 0.1],
#                  [-0.8,-0.5, 0.1] ])
# pos_ends  =arr([ [ 0.0,-1.0, 0.1],
#                  [-2.2, 1.5, 0.1] ])


pos_diffs  = pos_ends - pos_starts

pos_starts += bases; pos_ends +=bases;

###############################################################
################# Function Definition Part#####################
###############################################################


##########################################################

def get_link_lengths(robot):
     lengths = []
     prev = None
     for i in range(1,END_JOINT_IDX+1):
          curr = getLinkPos(robot, i)
          if prev is not None:
               dist = norm(curr - prev) 
               lengths.append(dist)
          prev = curr
     print(lengths)
     return lengths

import rrt_import

def main():
     global T_max
     init_simulation()
     robots = arr([p.loadURDF(ROBOT_PATH, bases[ind], startOri, useFixedBase=True) \
                         for ind in range(NUM_ROBOTS)])
     robots_class = list(map(lambda robot: StateRobot(robot, robots), robots))

     q_inits = arr([getNpIK(robot, END_JOINT_IDX, pos_starts[idx])  for idx, robot in enumerate(robots)]) 
     init_robots(robots, q_inits)
     input()

     # Draw Trajectories
     for i in ROBOT_IDXS:
          p.addUserDebugLine(list(pos_starts[i]), list(pos_ends[i]), lineColorRGB=[1,1,0], lineWidth=4)
          # drawLine(pos_starts[i], pos_ends[i], color=[1,1,0])


     position = []
     ## Start simulation
     lengths        = get_link_lengths(robots[0])
     crit_nums      = list(map(lambda x: int(np.floor(x/CRIT_DIST)),lengths))
     crit_nums[-1] += 1

     robot_pairs = list(combinations(robots,2))
     two_joints_list = [list(range(1,END_JOINT_IDX)), list(range(1,END_JOINT_IDX))]
     joint_pairs = list(product(*two_joints_list))
     T = 0
     print(crit_nums)
     path = rrt_import.plan(robots[0], robots, pos_ends[0])
     path1 = rrt_import.plan(robots[1], robots, pos_ends[1])
     rrt_import.show_path(robots[0], path)
     rrt_import.show_path(robots[1], path1)
     # if 1 : return
     while 1:
          try:
               norm_time = (T%T_max)/T_max
               pos_tars = pos_starts +  norm_time * pos_diffs


               # PD position control

               for idx, robot in enumerate(robots):
                    simulationRobot(robot, pos_tars[idx])
               # APF joint by joint
               for robot1,robot2 in robot_pairs:
                    for i,j in joint_pairs:
                         # applyPotentialForce(robot1, robot2, i, j, crit_nums)
                         applyPFjoint(robot1, robot2, i, j)
               # Show and reset simulation env
               # print(f"Time: {T%T_max}/{T_max}")
               T+=1;
               if T % T_max == 0: 
                    init_robots(robots, q_inits)
               p.stepSimulation()

               time.sleep(SIM_TIMESTEP * 1/240)

          except:
               raise



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
     p.setGravity(0, 0, -9.8)
     p.resetDebugVisualizerCamera(cameraDistance=4, cameraYaw=0, cameraPitch=-85, cameraTargetPosition=[1.2,0,0])




def simulationRobot(robot_id, pos_target):
     # p.setJointMotorControlArray(robot_id, [1,2], p.VELOCITY_CONTROL, forces=[0,0])
     q_des = getNpIK(robot_id, END_JOINT_IDX, pos_target)
     # Set joint control!
     joints = list(range(1, END_JOINT_IDX))
     Pscale = 0.005
     Dscale = 0.02
     Pvalue=arr([1.5,1.5,1.5])
     Dvalue=arr([3,  3,  3])
     p.setJointMotorControlArray(robot_id, joints, p.POSITION_CONTROL,
                                   targetPositions=q_des,
                                   positionGains=Pscale*Pvalue, velocityGains=Dscale*Dvalue) # Tune the gains



def applyPotentialForce(robot_1, robot_2, joint_index1, joint_index2, crit_nums): #it will be run inside the 2-layer-for-loop
     # print(joint_index1)
     # print(joint_index2)
     crits_list = [list(range(crit_nums[joint_index1-1])), list(range(crit_nums[joint_index2-1]))]
     crit_pairs = list(product(*crits_list))
     coeff = .05
     global D_MIN
     # D_MIN = 0.5
     maxForce = coeff/(D_MIN**2)*(1/D_MIN - 1/D_THRES)

     base_pos = arr([getLinkPos(robot_1, joint_index1),   getLinkPos(robot_2, joint_index2)])
     link_vec = arr([getLinkPos(robot_1, joint_index1+1), getLinkPos(robot_2, joint_index2+1)]) - base_pos
     direc = arr([link_vec[0]/norm(link_vec[0]), link_vec[1]/norm(link_vec[1])])

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

          p.applyExternalForce(robot_1, joint_index1, -force*direction, pos1, p.WORLD_FRAME)
          p.applyExternalForce(robot_2, joint_index2,  force*direction, pos2, p.WORLD_FRAME)
          # p.addUserDebugPoints(pointPositions = [base_pos[1]+direc[1]],
                              # pointColorsRGB  = [[0,0,1]], 
                              # pointSize = 5, lifeTime = 1)

          # drawLine(pos1, pos1 - force*direction, color=[1, 0.2, 0.2],lifeTime=0.2,width=3)
          # drawLine(pos2, pos2 + force*direction, color=[1, 0.2, 0.2],lifeTime=0.2,width=3)
     return 1

def applyPFjoint(robot_1, robot_2, joint_index1, joint_index2): # No-critical-point-version
     coeff = .2
     global D_MIN
     # D_MIN = 0.5
     maxForce = coeff/(D_MIN**2)*(1/D_MIN - 1/D_THRES)

     pos1 = getLinkPos(robot_1, joint_index1+1)
     pos2 = getLinkPos(robot_2, joint_index2+1)


     dist = norm(pos2-pos1)
     # print(f'Distance is {dist}')
     if dist > D_THRES: return
     if dist < D_LOCK: 
          return -1 # Deadlock occured
     if dist < D_MIN: 
          force = maxForce
     else: # D_MIN < d < D_THRES
          force = coeff/(dist**2)*(1/dist - 1/D_THRES)

     direction = (pos2-pos1)/dist # Direction unit vector


     p.applyExternalForce(robot_1, joint_index1, -force*direction, pos1, p.WORLD_FRAME)
     p.applyExternalForce(robot_2, joint_index2,  force*direction, pos2, p.WORLD_FRAME)

     drawLine(pos1, pos1 - force*direction, color=[1, 0.2, 0.2],lifeTime=0.2,width=3)
     drawLine(pos2, pos2 + force*direction, color=[1, 0.2, 0.2],lifeTime=0.2,width=3)
     return 1





if __name__=="__main__":
     main()

