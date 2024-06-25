import pybullet as p
import pybullet_data
import time
import numpy as np
from numpy import array as arr
from numpy.linalg import norm 
from itertools import combinations, product
# from .p_utils import init_robots, get_joint_angles, getLinkPos, getNpIK, drawLine, dummyfn, getRevoluteJointList, set_joint_angles
from p_utils import init_robots, get_joint_angles, getLinkPos, getNpIK, drawLine, dummyfn, getRevoluteJointList, set_joint_angles

import sys
basePath = "/home/irsl/libpython"
sys.path.append(f"{basePath}")
sys.path.append(f"{basePath}/interfaces/common")
sys.path.append(f"{basePath}/interfaces/impl")
ROBOT_PATH = f"{basePath}/indy7.urdf"

# from interfaces.device_socket_client import DeviceSocketClient
# from interfaces.config_socket_client import ConfigSocketClient
# from interfaces.control_socket_client import ControlSocketClient
# from interfaces.rtde_socket_client import RTDESocketClient
# import common as common

import rclpy
from rclpy.node import Node

from std_msgs.msg import String

import rrt_import
from robot_class import StateRobot


###############################################################
################# Parameter Definition Part####################
###############################################################
END_JOINT_IDX = 6

D_THRES   = 0.6
D_MIN     = 0.3
D_LOCK    = 0.1
CRIT_DIST = 0.15
SELF_CRIT_DIST = 0.15



class RobotInfoPublisher(Node):
    def __init__(self, robotid, base, yaw):
        super().__init__('StatePublisher')
        self.robotid = robotid
        self.base   = base
        self.yaw    = yaw
        self.publisher = self.create_publisher(String, f'/robot{robotid}', 30)
        print(f"Publish to {robotid}")
    def publish(self, deadlock = False, jointangle = None):
        global joint_states
        msg = String()
        q_ = str(get_joint_angles(self.robotid))[1:-1]
        msg.data = f"{self.robotid}||{self.base[0]}||{self.base[1]}||{self.yaw}||{deadlock}||{q_}"
        #update myself
        info = msg.data.split("||")
        joint_states[self.robotid] = info

        self.publisher.publish(msg)
        # print(f"PPPPPP")
        # print(f"PPPPPP{msg.data}")
        # print(f"_________________")
        # print(f"_________________")

class RobotIdSubscriber(Node):
    def __init__(self, targetid):
        super().__init__(f'robot_info_subs')
        self.subscription = self.create_subscription(String, f'/robot{targetid}', self.callback, 30)
        self.last_time = None
        print(f"Subs to {targetid}")

    def callback(self, msg):
        global joint_states
        info = msg.data.split("||")
        index = int(info[0])
        joint_states[index] = info


        # time_now = time.time()
        # if self.last_time is not None:
        #     print(f"Updated in {time_now - self.last_time}sec")
        # self.last_time = time_now
        # print(f"HEARD {info}")
        # print(f"_________________")
        # print(f"_________________")





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




def status_collision(robots):
    pair_candidate = list(combinations(robots,2))
    for robot1, robot2 in pair_candidate:
        pp = p.getContactPoints(robot1, robot2)
        if len(pp) > 0: return True
    return False

def makeRobotsNaive(robots): #In Simulation..
    naive_pose = arr([0, 0, 0, 0, 0, 0])
    rb = np.repeat(naive_pose, len(robots))
    rb = np.reshape(rb, (6,len(robots))).T
    init_robots(robots, rb, start_index = 0)

def makeTrajectoryJointSpace(robot, traj, parOri):
    jointTraj=[]
    # for idx, robot in enumerate(robots):
    for wp_idx, waypoint in enumerate(trajs[idx]):
        orientation = None if wp_idx !=0 else parOri[idx]
        transformed = getNpIK(robot, END_JOINT_IDX, traj[wp_idx], orientation)
        jointTraj.append(transformed)
    return jointTraj

def drawWayPoints(robot, base, jointTraj):
    p.addUserDebugText(f"Robot {robot}", base + arr([0,0,1.5]), textColorRGB = [0,0,0], textSize = 1, lifeTime=15)
    prevpose = None

    for i in range(len(jointTraj)):
        set_joint_angles (robot, jointTraj[i], endJointIndex=END_JOINT_IDX)
        pose = getLinkPos(robot, END_JOINT_IDX)
        if prevpose is not None:
            p.addUserDebugLine(prevpose, pose, lineColorRGB=[.9, 1-0.12*robot, 0.25*robot], lineWidth=4)#, lifeTime = 10)
        prevpose = pose

def makeCriticalElements(robots, robot, bases):
    lengths      = get_link_lengths(robots[0])
    crit_nums    = list(map(lambda x: int(np.ceil(x/CRIT_DIST)),lengths))
    crit_nums.append(1)
    self_crit_nums    = list(map(lambda x: int(np.ceil(x/SELF_CRIT_DIST)),lengths))
    self_crit_nums.append(1)

    near_robots = []
    global joint_states
    base = parse_string(joint_states[robot])[0]
    for robotid, robot_candidate in enumerate(robots):
        if robotid == robot: continue
        if norm(arr(base) - arr(bases[robot_candidate])) < 2*0.85: #0.85 is workspace each
            near_robots.append(robotid)

    two_joints_list = [list(range(1,END_JOINT_IDX+1)), list(range(1,END_JOINT_IDX+1))]
    joint_pairs = list(product(*two_joints_list))
    return crit_nums, self_crit_nums, near_robots, joint_pairs



def initialize_rosnodes(num_robots, robot, base, yaw):
    global joint_states
    rclpy.init()    
    joint_states = [None for _ in range(num_robots)]
    joint_states[robot] = [robot, base[0], base[1], yaw, 'False', str(get_joint_angles(robot))[1:-1]]
    # print(joint_states[robot])
    # for id_ in range(num_robots):
        # if id_ == robot: continue
    # subscriber = RobotIdSubscriber(robot)
    subscriberList = []
    for robotid in range(num_robots):
        if robotid == robot: continue
        subscriberList.append(RobotIdSubscriber(robotid))
    # print("Finish initializer")
    return subscriberList

def subs_spin_all(subsList):
    for sub in subsList:
        rclpy.spin_once(sub, timeout_sec = 0)


def wait_other_robots(publisher, subscriberList):
    global joint_states
    while joint_states.count(None) > 0:
        publisher.publish()
        print("AA")
        subs_spin_all(subscriberList)
        # rclpy.spin_once(subscriber, timeout_sec = 0)
        print("WAITING FOR OTHER NODES")
        time.sleep(0.3)
    print("FOUND!")

def parse_string(joint_state):
    base = [float(joint_state[1]), float(joint_state[2]), 0] 
    yaw  =  float(joint_state[3])
    deadlock = bool(joint_state[4])
    jointangleString = joint_state[5].split(',')
    jointangle = list(map(lambda angle: float(angle), jointangleString))
    return base, yaw, deadlock, jointangle


def update_robot_states(robots, robot):
    global joint_states
    for robotid in robots:
        if robotid == robot: continue
        base, yaw, deadlock, jointangle = parse_string(joint_states[robotid])
        set_joint_angles(robotid, jointangle)

MULTIPLIER = 1
import time
def testpart(num_robots, robot, traj, base, yaw, jointSpace = False, controller = None, rtde = None):
    global joint_states
    print(yaw)
    subscriberList = initialize_rosnodes(num_robots, robot, base, yaw)
    publisher  = RobotInfoPublisher(robot, base, yaw)
    wait_other_robots(publisher, subscriberList)

    parOri   = p.getQuaternionFromEuler([np.pi, 0, yaw])

    robots = list(range(num_robots)); q_inits = []; q_ends=[]; bases=[]
    print(joint_states)
    for idx, joint_state in enumerate(joint_states):
        base_, yaw_, deadlock_, jointangle_ = parse_string(joint_state)
        q_inits.append(jointangle_)
        bases.append(base_)
        baseOrientation = p.getQuaternionFromEuler( [0,0,yaw_] )
        p.resetBasePositionAndOrientation(idx, base_, baseOrientation)
       # ori = p.getQuaternionFromEuler([np.pi,0,idx*np.pi])

    # To resolve IK
    # makeRobotsNaive(robots)
    jointTraj = traj if jointSpace is True else makeTrajectoryJointSpace(robot, traj, parOri)
    robot_class = StateRobot(robot, robots, jointTraj)
    robots = arr(robots)

    init_robots(robots, q_inits, start_index = 0)
    p.stepSimulation()

    # Draw Trajectories
    drawWayPoints(robot, base, jointTraj)

    ## Start simulation
    crit_nums, self_crit_nums, near_robots, joint_pairs = makeCriticalElements(robots, robot, bases)

    global timestep
    timestep = 0
    if rtde is not None:
        q_now = rtde.GetControlData()['q']
        q = arr(q_inits[0]) * 180 / np.pi
        controller.MoveJ(jstart=q_now , jtarget = q, blending_type=common.Property.BlendingType.OVERRIDE)
        input("Make sure that robot is stopped")

    prevtime = time.time()
    # input()
    while 1:
        try:
            # if timestep % 2 == 0:
            start = time.time()
            subs_spin_all(subscriberList)
            # rclpy.spin_once(subscriber, timeout_sec=0)
            update_robot_states(robots, robot)
            # print(f'Elapsed time:"{time.time()-start} second')

            tar = robot_class.get_target(timestep)
            # print(tar)
            # print(get_joint_angles(robot)[:-1])
            # print("_________")

            if tar is not None: 
                simulationRobot(robot, tar)

            # APF joint by joint
            for near_robot in near_robots:
                _, _, near_deadlock, _ = parse_string(joint_states[near_robot])
                if near_deadlock == 'True': continue
                for i,j in joint_pairs:
                    applyPotentialForce(robot, near_robot, i, j, crit_nums, robot_class)
                    dummyfn()

            if robot_class.deadlock:
                applySelfPF(robot_class, self_crit_nums)

            timestep += 1; 
            # if timestep % 2 == 0:
            publisher.publish(robot_class.deadlock, get_joint_angles(robot)[:-1])

            if controller is not None and timestep%10 == 0:
                q_now = rtde.GetControlData()['q']
                q = arr(get_joint_angles(robot)[:-1], dtype = np.float64) * 180 / np.pi
                controller.MoveJ(jstart=q_now , jtarget = q, blending_type=common.Property.BlendingType.OVERRIDE)
                # input()


            # if timestep % 100 == 0 and robot_class.done:
                # return 1
            # if timestep == T_max: return 0
            p.stepSimulation()

            nowtime = time.time()
            # print(f"T I M E {nowtime-prevtime}")
            prevtime=nowtime
            time.sleep(MULTIPLIER*1/240)

        except:
          raise




def init_simulation():
    # Init sim
    # physicsClient = p.connect(p.DIRECT, options = "--width=800 --height=800")#or p.DIRECT for non-graphical version
    physicsClient = p.connect(p.GUI, options = "--width=800 --height=800")#or p.DIRECT for non-graphical version
    # Set up the visualizer
    p.configureDebugVisualizer(p.COV_ENABLE_WIREFRAME, 0)
    p.configureDebugVisualizer(p.COV_ENABLE_SHADOWS, 0)
    # p.configureDebugVisualizer(p.COV_ENABLE_GUI, 1)
    p.setRealTimeSimulation(0)
    p.setAdditionalSearchPath(pybullet_data.getDataPath())
    p.setTimeStep(MULTIPLIER*1/240)
    # p.setGravity(0, 0, -9.8)
    p.configureDebugVisualizer(p.COV_ENABLE_GUI,0)
    center = 0.4
    p.resetDebugVisualizerCamera(cameraDistance=2, cameraYaw=10, cameraPitch=-65, cameraTargetPosition=[center, center, 0.5])


def simulationRobot(robot, joint_target):
    # p.setJointMotorControlArray(robot, [1,2], p.VELOCITY_CONTROL, forces=[0,0])
    # q_des = getNpIK(robot, END_JOINT_IDX, pos_target)
    # Set joint control!
    joints = getRevoluteJointList(robot)
    joint_target = [joint_target[q] for q in joints]
    Pscale = 0.01
    Dscale = 0.2
    Pvalue=arr([3, 3, 3, 2, 2, 2])
    Dvalue=arr([3, 3, 3, 2, 2, 2])
    p.setJointMotorControlArray(robot, joints, p.POSITION_CONTROL,
            targetPositions=joint_target,
            positionGains=Pscale*Pvalue, velocityGains=Dscale*Dvalue) # Tune the gains



def applyPotentialForce(robot, near_robot, joint_index1, joint_index2, crit_nums, robot_class): #it will be run inside the 2-layer-for-loop
    # print(joint_index1)
    crits_list = [list(range(crit_nums[joint_index1-1])), list(range(crit_nums[joint_index2-1]))]
    crits = (crit_nums[joint_index1-1], crit_nums[joint_index2-1])
    crit_pairs = list(product(*crits_list))
    coeff = 40
    global D_MIN
    D_MIN = 0.5
    maxForce = coeff/(D_MIN**2)*(1/D_MIN - 1/D_THRES)


    joint_pos = arr([getLinkPos(robot, joint_index1),   getLinkPos(near_robot, joint_index2)])
    link_vec = arr([getLinkPos(robot,      min(joint_index1+1, END_JOINT_IDX)),
                    getLinkPos(near_robot, min(joint_index2+1, END_JOINT_IDX))]) - joint_pos
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
        p.applyExternalForce(robot, joint_index1, -force*direction, pos1, p.WORLD_FRAME)
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

        joint_pos = arr([joint_poses[joint1],    joint_poses[joint2]])
        link_vec = arr([joint_poses[next_joint1], joint_poses[next_joint2]]) - joint_pos
        # direc = arr([np.zeros_like(link_vec[0]) if norm(link_vec[0]) == 0 else link_vec[0]/norm(link_vec[0]),
        #      np.zeros_like(link_vec[1]) if norm(link_vec[1]) == 0 else link_vec[1]/norm(link_vec[1])]) 


        jointpair_crit_index = [list(range(crit_nums[joint1])), list(range(crit_nums[joint2]))]
        jointpair_crit_index = list(product(*jointpair_crit_index))
        crits = (crit_nums[joint1-1], crit_nums[joint2-1])
        
        for crit_1, crit_2 in jointpair_crit_index:
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
        randStart = randXYZ(yaw) + base

        while True:
            randEnd = randXYZ(yaw) + base
            if norm(randEnd  - randStart) > 0.3: break
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




class robotinfos():
    def __init__(self, num_robots):
        self.num_robots = num_robots
        self.found       = [False for _ in range(num_robots)]
        self.deadlock    = [False for _ in range(num_robots)]
        self.bases       = [None  for _ in range(num_robots)]
        self.jointStates = [None  for _ in range(num_robots)]
        self.myId = None

    def getRobotId(self,base, publisher):
        for i, status in enumerate(self.found):
            if status is False:
                self.myId = i
                self.found[i] = True
                self.bases[i] = base
        return self.myId








def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--numrobot', default=None, type=int)
    parser.add_argument('--ip', default='192.168.151.248')
    parser.add_argument('--basex', default=None, type=float)
    parser.add_argument('--basey', default=None, type=float)
    parser.add_argument('--ori', default=0, type=float)
    parser.add_argument('--robotid', default=0, type=int)
    args = parser.parse_args()
    print('Args:', args)

    num_robots = 2 if args.numrobot is None else args.numrobot
    base = arr([args.basex, args.basey, 0])
    ip   = args.ip
    yaw  = args.ori
    robot= args.robotid
    assert robot >= 0 and robot < num_robots


    # startOri = [p.getQuaternionFromEuler([0, 0, yaw])]
    init_simulation()
    for idx in range(num_robots):
        p.loadURDF(ROBOT_PATH, [idx, 0, 0], useFixedBase=True)
    p.loadURDF("plane.urdf",[0, 0, 0])

    np.set_printoptions(precision=3)

    print(base)
    # yaws = [0 for i in range(num_robots)]
    traj =  arr([   # Control my robot only
                [0,0,-90,0,-90,0],
                [0,0,0,0,0,0],
                [20,20,-90,20,-90,0],
                [0,30,0,30,0,0],
                [0,0,-90,0,-90,0],
                [0,20,0,20,0,0],
                ],dtype=np.float64)
    traj *=  np.pi / 180 # Degree to radian
    controller,rtdata = None, None

    success = testpart(num_robots, robot, traj, base, yaw, jointSpace = True, controller = controller, rtde = rtdata)

import argparse

if __name__=="__main__":
    main()
