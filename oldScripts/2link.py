import pybullet as p
import pybullet_data
import time
import numpy as np

SIM_TIMESTEP = 1

# Init sim
physicsClient = p.connect(p.GUI)#or p.DIRECT for non-graphical version
# Set up the visualizer
p.configureDebugVisualizer(p.COV_ENABLE_WIREFRAME, 0)
p.configureDebugVisualizer(p.COV_ENABLE_SHADOWS, 0)
p.configureDebugVisualizer(p.COV_ENABLE_GUI, 1)
p.setRealTimeSimulation(0)
p.setTimeStep(SIM_TIMESTEP)

p.setGravity(0, 0, -9.8)

# load robot
startOrientation = p.getQuaternionFromEuler([0,0,0])
base_1 = np.array([0,   0, 0])
base_2 = np.array([2, -0.5, 0])
robot1 = p.loadURDF("./2linkarm.urdf", base_1, startOrientation, useFixedBase=True)
robot2 = p.loadURDF("./2linkarm.urdf", base_2, startOrientation, useFixedBase=True)

pos_start1 = base_1 + [ 1.5, 1.2, 0.1]
pos_end1   = base_1 + [ 0.0,-1.0, 0.1]
pos_diff1  = pos_end1 - pos_start1

pos_start2 = base_2 + [-0.8, -0.5, 0.1]
pos_end2   = base_2 + [-1, 1.5, 0.1]
pos_diff2  = pos_end2 - pos_start2

# for i in range(p.getNumJoints(robot1)):
#     print(p.getJointInfo(robot1, i))

END_JOINT_INDEX = 3

for i in range(END_JOINT_INDEX):
     p.resetJointState(robot1, i, 0)
     p.resetJointState(robot2, i, 0)
# p.setJointMotorControlArray(robot1, [0,1], p.POSITION_CONTROL, targetPositions=[0,0], positionGains=[0.01,1], velocityGains=[0.1,0.99])


# User param part
# des_value1 = p.addUserDebugParameter("Value: position1", -1, 1, 0)
# des_value2 = p.addUserDebugParameter("Value: position2", -1, 1, 0)
# Fx_ = p.addUserDebugParameter("Value: Forcex", -1, 1, 0)
# Fy_ = p.addUserDebugParameter("Value: Forcey", -1, 1, 0)
# maxLim = 2
# des_x_ = p.addUserDebugParameter("Value: des x", -maxLim, maxLim, 0)
# des_y_ = p.addUserDebugParameter("Value: des y", -maxLim, maxLim, 0)



# Start from (-1,-1)


def getNpLinkPose(robot_id, joint_index):
     return np.array(p.getLinkState(robot_id, joint_index)[0])
def getNpInverseKinematics(robot_id, joint_index, position):
     return np.array(p.calculateInverseKinematics(robot_id, joint_index, position))

q  = getNpInverseKinematics(robot1, END_JOINT_INDEX, pos_start1)
q2 = getNpInverseKinematics(robot2, END_JOINT_INDEX, pos_start2)


def init(robot_id, q_start):
     for i in range(END_JOINT_INDEX-1):
          p.resetJointState(robot_id, i+1, q_start[i], targetVelocity=0)


init(robot1, q)
init(robot2, q2)

T=0; T_max = 300
# normalized = list(T_/T_max for T_ in range(T_max))

p.addUserDebugLine(pos_start1, pos_end1, lineColorRGB=[1,1,0],lineWidth=3)
p.addUserDebugLine(pos_start2, pos_end2, lineColorRGB=[1,0,1],lineWidth=3)


def simulationRobot(robot_id, pos_target):
     # p.setJointMotorControlArray(robot_id, [1,2], p.VELOCITY_CONTROL, forces=[0,0])


     q_des  = getNpInverseKinematics(robot_id, 3, pos_target)
     # Set joint control!
     p.setJointMotorControlArray(robot_id, [1,2], p.POSITION_CONTROL,
                                   targetPositions=q_des,
                                   positionGains=[.008,.008], velocityGains=[.06,.06]) # Gain should be tuned?

     # Apply Additional Force
     Fx = .3*np.cos(5*T/T_max*6.28)
     Fy = .3*np.sin(3*T/T_max*6.28)
     Fend = np.array([Fx,Fy,0]) # Force vector
     Fend = np.array([0,0,0]) # Force vector

     elbow_pose = getNpLinkPose(robot_id,2)
     ee_pose = getNpLinkPose(robot_id,3)

     p.applyExternalForce(robot_id, 2, Fend, elbow_pose, p.WORLD_FRAME)
     p.applyExternalForce(robot_id, 3, Fend, ee_pose, p.WORLD_FRAME)
     p.addUserDebugLine(elbow_pose, elbow_pose + 5*Fend, lineColorRGB=[1,0,0],lifeTime=0.1,lineWidth=5)
     p.addUserDebugLine(ee_pose, ee_pose + 5*Fend, lineColorRGB=[1,0,0],lifeTime=0.1,lineWidth=5)




def applyPotentialForce(robot_1, robot_2, joint_index1, joint_index2): #it will be run inside the 2-layer-for-loop
     scale = .1
     coeff=.5; d_limit = .7

     pos1 = getNpLinkPose(robot_1, joint_index1)
     pos2 = getNpLinkPose(robot_2, joint_index2)
     distance_cartesian = np.linalg.norm(pos2-pos1)
     
     # print(f'Distance is {distance_cartesian}')
     if distance_cartesian > d_limit: return
     d = distance_cartesian
     maxForce = .05
     force = min(maxForce, coeff/(d**2)*(1/d - 1/d_limit)) 

     direction = (pos2-pos1)/d # Direction unit vector
     print(force)

     p.applyExternalForce(robot_1, joint_index1, -force*direction, pos1, p.WORLD_FRAME)
     p.applyExternalForce(robot_2, joint_index2,  force*direction, pos2, p.WORLD_FRAME)
     p.addUserDebugLine(pos1, pos1 - 10*force*direction, lineColorRGB=[1,0,0],lifeTime=0.05,lineWidth=5)
     p.addUserDebugLine(pos2, pos2 + 10*force*direction, lineColorRGB=[1,0,0],lifeTime=0.05,lineWidth=5)



input()
while True:
     try:
          norm_time = (T%T_max)/T_max

          pos_tar1 = pos_start1 +  norm_time * pos_diff1
          pos_tar2 = pos_start2 +  norm_time * pos_diff2

          # Apply force to last link
          print(f"Time: {T%T_max}/{T_max}")
          for i in range(2,4):
               for j in range(i,4):
                    applyPotentialForce(robot1, robot2, i, j)
                    print(f"{i}, {j}")

          simulationRobot(robot1, pos_tar1)
          simulationRobot(robot2, pos_tar2)

          T+=1;
          if T % T_max == 0: 
               init(robot1, q)
               init(robot2, q2)
          p.stepSimulation()



          time.sleep(SIM_TIMESTEP * 1/240)

     except:
          raise