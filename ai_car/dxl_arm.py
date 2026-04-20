#!/usr/bin/env python3

import time
from dynamixel_sdk import *

BAUD_RATE = 1000000
DXL_PORT = PortHandler('/dev/ttyUSB0')

ARM_JOINT_1 = 11            # 812 ~ 512     0 - 90
ARM_JOINT_2 = 12            # 512 - 212     0 - 90
ARM_TOOL = 13               # 450 - 612     0 - 90

AX_TICK2RAD = 0.005113269      # 300 / 1024 / 180 * pi
AX_TICK2DEG = 0.29296875       # 300 / 1024

def to_int32(value):
    if value > 0x7FFFFFFF: return value - 0x100000000
    return value

def to_uint32(value):
    return int(value) & 0xFFFFFFFF

class Arm_Controller():
    def __init__(self, port):
        pass

    def reset(self):
        pass
        
    def set_torque(self, set_data):
        set_data = to_uint32(set_data)

    def set_joint(self, set_data):
        set_data = [to_uint32(x) for x in set_data]

    def set_tool(self, set_data):
        set_data = to_uint32(set_data)

    def get_arm(self):
        get_data = [0, 0, 0]

        return get_data

if __name__ == '__main__':
    print('dynamixel arm controller')
    try:
        pass

    except KeyboardInterrupt:
        pass