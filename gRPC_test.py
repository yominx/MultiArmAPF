from interfaces.device_socket_client import DeviceSocketClient
from interfaces.config_socket_client import ConfigSocketClient
from interfaces.control_socket_client import ControlSocketClient
from interfaces.rtde_socket_client import RTDESocketClient

import common as common
import time

ip = '192.168.151.248'
control = ControlSocketClient(ip)
device = DeviceSocketClient(ip)
config = ConfigSocketClient(ip)
rtde = RTDESocketClient(ip)

control.GetVariableNameList()
control.GetIntVariable()





int_var = dict()
int_var['addr'] = 0
int_var['value'] = 1
control.SetIntVariable([int_var])

a = rtde.GetControlData()
print(a)
# rtde.GetControlData()
t_now =rtde.GetControlData()['p'] # task pos
q_now = rtde.GetControlData()['q'] # joint pos

origin = [0,0,0,0,0,0]
rest_pose = [0,0,-90,0,-90,0]
target_pos1 = [1,1,0,1,0,1]
target_pos2 = [2,2,89,2,89,2]
target_pos3 = [1,1,60,1,60,1]
vel_ratio = 50
acc_ratio = 50
control.MoveJ(jstart=q_now , jtarget = origin, blending_type=common.Property.BlendingType.OVERRIDE, vel_ratio=vel_ratio, acc_ratio=acc_ratio)
time.sleep(10)
control.MoveJ(jstart=q_now , jtarget = rest_pose, blending_type=common.Property.BlendingType.OVERRIDE, vel_ratio=vel_ratio, acc_ratio=acc_ratio)
input()
print("end")