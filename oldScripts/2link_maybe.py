import pybullet as p
import pybullet_data
import time
import numpy as np
from numpy import array as arr
from itertools import combinations, product
###############################################################
################# Parameter Definition Part####################
###############################################################
END_JOINT_IDX = 3
NUM_ROBOTS = 2

JOINT_IDXS = arr([range(END_JOINT_IDX)])
ROBOT_IDXS = arr(range(NUM_ROBOTS))

ROBOT_PATH = "./2linkarm.urdf"

T_max = 400
SIM_TIMESTEP = 1
D_THRES = 0.8
D_MIN = 0.6
D_LOCK = 0.3

###############################################################
################# Loading Robots Part #########################
###############################################################

# Load robot poses & task poses
startOri = p.getQuaternionFromEuler([0,0,0])
bases = arr([  [0,   0, 0], 
               [2.0, -0.5, 0]  ])
               # [2.5, -0.5, 0]  ])
assert(len(bases) == NUM_ROBOTS)

# Define tasks

pos_starts=arr([ [ 2.1, 0.5, 0.1],
                 [-0.4,-0.5, 0.1] ])
pos_ends  =arr([ [ 0.0,-1.0, 0.1],
                 [-1.5, 1.5, 0.1] ])
# pos_starts=arr([ [ 1.5, 1.2, 0.1],
#                  [-0.8,-0.5, 0.1] ])
# pos_ends  =arr([ [ -0.6,-1.0, 0.1],
#                  [-2.2, 1.5, 0.1] ])


pos_diffs  = pos_ends - pos_starts

pos_starts += bases; pos_ends +=bases;

###############################################################
################# Function Definition Part#####################
###############################################################

def init_robots(robots, q_inits):
     for rb_idx, robot in enumerate(robots):
          for joint_i in range(END_JOINT_IDX-1):
               p.resetJointState(robot, joint_i+1, q_inits[rb_idx][joint_i], targetVelocity=0)

def getLinkPos(robot_id, joint_index):
     return arr(p.getLinkState(robot_id, joint_index)[0])
def getNpIK(robot_id, joint_index, position):
     return arr(p.calculateInverseKinematics(robot_id, joint_index, position))

def drawLine(pos_start, pos_end, color,lifeTime=100,width = 3):
     p.addUserDebugLine(pos_start, pos_end, lineColorRGB=color, lifeTime=lifeTime, lineWidth=width)

def dummyfn():
     return
##########################################################



def main():
     global T_max
     init_simulation()
     robots = arr([p.loadURDF(ROBOT_PATH, bases[ind], startOri, useFixedBase=True) \
                         for ind in range(NUM_ROBOTS)])

     q_inits = arr([getNpIK(robot, END_JOINT_IDX, pos_starts[idx])  for idx, robot in enumerate(robots)]) 
     init_robots(robots, q_inits)
     # Draw Trajectories
     print(pos_starts[0])
     print(pos_ends[0])
     for i in ROBOT_IDXS:
          p.addUserDebugLine(list(pos_starts[i]), list(pos_ends[i]), lineColorRGB=[1,1,0], lineWidth=4)
          # drawLine(pos_starts[i], pos_ends[i], color=[1,1,0])


     for i in JOINT_IDXS:
          print(getLinkPos(robots[0], i))



     ## Start simulation
     robot_pairs = list(combinations(robots,2))
     two_joints_list = [list(range(2,END_JOINT_IDX+1)), list(range(2,END_JOINT_IDX+1))]
     joint_pairs = list(product(*two_joints_list))
     T = 0
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
                         applyPotentialForce(robot1, robot2, i, j)
                         dummyfn()

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




def simulationRobot(robot_id, pos_target):
     # p.setJointMotorControlArray(robot_id, [1,2], p.VELOCITY_CONTROL, forces=[0,0])
     q_des = getNpIK(robot_id, END_JOINT_IDX, pos_target)
     # Set joint control!
     joints = list(range(1, END_JOINT_IDX))
     Pscale = 0.005
     Dscale = 0.02
     Pvalue=arr([1.5,1.5])
     Dvalue=arr([3,  3])
     p.setJointMotorControlArray(robot_id, joints, p.POSITION_CONTROL,
                                   targetPositions=q_des,
                                   positionGains=Pscale*Pvalue, velocityGains=Dscale*Dvalue) # Tune the gains



def applyPotentialForce(robot_1, robot_2, joint_index1, joint_index2): #it will be run inside the 2-layer-for-loop
     coeff = 1
     global D_MIN
     # D_MIN = 0.5
     maxForce = coeff/(D_MIN**2)*(1/D_MIN - 1/D_THRES)

     pos1 = getLinkPos(robot_1, joint_index1)
     pos2 = getLinkPos(robot_2, joint_index2)
     dist = np.linalg.norm(pos2-pos1) # Distance in cartesian space
     
     # print(f'Distance is {dist}')
     if dist > D_THRES: return 1
     if dist < D_LOCK: 
          return -1 # Deadlock occured
     if dist < D_MIN: 
          force = maxForce
     else: # D_MIN < d < D_THRES
          force = coeff/(dist**2)*(1/dist - 1/D_THRES)

     direction = (pos2-pos1)/dist # Direction unit vector

     p.applyExternalForce(robot_1, joint_index1, -force*direction, pos1, p.WORLD_FRAME)
     p.applyExternalForce(robot_2, joint_index2,  force*direction, pos2, p.WORLD_FRAME)
     drawLine(pos1, pos1 - force*direction, color=[1, 0.2, 0.2],lifeTime=0.07,width=3)
     drawLine(pos2, pos2 + force*direction, color=[1, 0.2, 0.2],lifeTime=0.07,width=3)
     return 1

if __name__=="__main__":
     main()

