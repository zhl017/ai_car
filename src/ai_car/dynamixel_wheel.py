#!/usr/bin/env python3

import numpy as np
from dynamixel_sdk import *

BAUD_RATE = 1000000
DXL_PORT = PortHandler('/dev/ttyUSB0')

XM_OPERATING_MODE_ADDRESS = 11
XM_TORQUE_ADDRESS = 64
XM_GOAL_VELOCITY_ADDRESS = 104
XM_PRESENT_VELOCITY_ADDRESS = 128
XM_PRESENT_POSITION_ADDRESS = 132

LEFT_ID = 1
RIGHT_ID = 2
REAR_LEFT_ID = 3
REAR_RIGHT_ID = 4

XM_TICK2RAD = 0.001533981

class Wheel_Controller():
    def __init__(self, port):
        self.port = port
        self.packet = PacketHandler(1.0)

        self.reset()

    def reset(self):
        self.set_torque(0)

        self.packet.write1ByteTxRx(self.port, LEFT_ID, XM_OPERATING_MODE_ADDRESS, int(3))
        self.packet.write1ByteTxRx(self.port, RIGHT_ID, XM_OPERATING_MODE_ADDRESS, int(3))
        self.packet.write1ByteTxRx(self.port, REAR_LEFT_ID, XM_OPERATING_MODE_ADDRESS, int(3))
        self.packet.write1ByteTxRx(self.port, REAR_RIGHT_ID, XM_OPERATING_MODE_ADDRESS, int(3))

        self.packet.write1ByteTxRx(self.port, LEFT_ID, XM_OPERATING_MODE_ADDRESS, int(1))
        self.packet.write1ByteTxRx(self.port, RIGHT_ID, XM_OPERATING_MODE_ADDRESS, int(1))
        self.packet.write1ByteTxRx(self.port, REAR_LEFT_ID, XM_OPERATING_MODE_ADDRESS, int(1))
        self.packet.write1ByteTxRx(self.port, REAR_RIGHT_ID, XM_OPERATING_MODE_ADDRESS, int(1))

        self.set_torque(1)

    def set_torque(self, set_data):
        set_data = np.int32(set_data)
        # print(set_data)
        self.packet.write1ByteTxRx(self.port, LEFT_ID, XM_TORQUE_ADDRESS, set_data)
        self.packet.write1ByteTxRx(self.port, RIGHT_ID, XM_TORQUE_ADDRESS, set_data)
        self.packet.write1ByteTxRx(self.port, REAR_LEFT_ID, XM_TORQUE_ADDRESS, set_data)
        self.packet.write1ByteTxRx(self.port, REAR_RIGHT_ID, XM_TORQUE_ADDRESS, set_data)

    def set_velocity(self, set_data):
        set_data = np.int32(list(set_data))
        # print(set_data)
        self.packet.write4ByteTxRx(self.port, LEFT_ID, XM_GOAL_VELOCITY_ADDRESS, set_data[0])
        self.packet.write4ByteTxRx(self.port, RIGHT_ID, XM_GOAL_VELOCITY_ADDRESS, set_data[1])
        self.packet.write4ByteTxRx(self.port, REAR_LEFT_ID, XM_GOAL_VELOCITY_ADDRESS, set_data[0])
        self.packet.write4ByteTxRx(self.port, REAR_RIGHT_ID, XM_GOAL_VELOCITY_ADDRESS, set_data[1])

    def get_position(self):
        get_data = [0,0,0,0]
        get_data[0], result, error = self.packet.read4ByteTxRx(self.port, LEFT_ID, XM_PRESENT_POSITION_ADDRESS)
        get_data[1], result, error = self.packet.read4ByteTxRx(self.port, RIGHT_ID, XM_PRESENT_POSITION_ADDRESS)
        get_data[2], result, error = self.packet.read4ByteTxRx(self.port, REAR_LEFT_ID, XM_PRESENT_POSITION_ADDRESS)
        get_data[3], result, error = self.packet.read4ByteTxRx(self.port, REAR_RIGHT_ID, XM_PRESENT_POSITION_ADDRESS)
        
        get_data = np.int32(get_data)
        return get_data
    
if __name__ == '__main__':

    print('dynamixel wheel controller')
    DXL_PORT.openPort()
    DXL_PORT.setBaudRate(BAUD_RATE)

    wheel = Wheel_Controller(DXL_PORT)

    wheel.set_velocity([-20, -20])

    while True:
        print((wheel.get_position()))

