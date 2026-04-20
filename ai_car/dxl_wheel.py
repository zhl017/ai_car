#!/usr/bin/env python3

import time
from dynamixel_sdk import *

BAUD_RATE = 1000000
DXL_PORT = PortHandler('/dev/ttyUSB0')

LEFT_ID = 1
RIGHT_ID = 2
REAR_LEFT_ID = 3
REAR_RIGHT_ID = 4

XM_TICK2RAD = 0.001533981

def to_int32(value):
    if value > 0x7FFFFFFF: return value - 0x100000000
    return value

def to_uint32(value):
    return int(value) & 0xFFFFFFFF

class Wheel_Controller():
    def __init__(self, port):
        pass

    def reset(self):
        pass

    def set_torque(self, set_data):
        set_data = to_uint32(set_data)

    def set_wheel(self, set_data):
        set_data = [to_uint32(x) for x in set_data]

    def get_wheel(self):
        get_data = [0,0,0,0]

        get_data = [to_int32(x) for x in get_data]
        return get_data
    
if __name__ == '__main__':
    print('dynamixel wheel controller')
    try:
        pass

    except KeyboardInterrupt:
        pass