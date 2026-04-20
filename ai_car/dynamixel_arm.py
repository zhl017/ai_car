#!/usr/bin/env python3

import numpy as np
import time
from dynamixel_sdk import *

BAUD_RATE = 1000000
DXL_PORT = PortHandler('/dev/ttyUSB0')

AX_LED_ADDRESS = 25
AX_TORUQE_ADDRESS = 24
AX_GOAL_POSITION = 30
AX_MOVING_SPEED = 32
AX_PRESENT_POSITION = 36

ARM_JOINT_1 = 11            # 812 ~ 512     0 - 90
ARM_JOINT_2 = 12            # 512 - 212     0 - 90
ARM_TOOL = 13               # 450 - 612    -0.32 - 0.51

HOME_POSE = [90, 90]

SLEEP_POSE = [0, 0]
SLEEP_TOOl = 10

AX_TICK2RAD = 0.005113269      # 300 / 1024 / 180 * pi
AX_TICK2DEG = 0.29296875       # 300 / 1024

class Arm_Controller():
    def __init__(self, port):
        self.port = port
        self.packet = PacketHandler(1.0)

        self.packet.write2ByteTxRx(self.port, ARM_JOINT_1, AX_MOVING_SPEED, int(100))
        self.packet.write2ByteTxRx(self.port, ARM_JOINT_2, AX_MOVING_SPEED, int(100))
        self.packet.write2ByteTxRx(self.port, ARM_TOOL, AX_MOVING_SPEED, int(100))
        time.sleep(1)

        self.reset()

    def reset(self):
        self.set_joint(SLEEP_POSE)
        self.set_tool(SLEEP_TOOl)
        
    def set_torque(self, set_data):
        set_data = np.int32(set_data)
        self.packet.write1ByteTxRx(self.port, ARM_JOINT_1, AX_TORUQE_ADDRESS, set_data)
        self.packet.write1ByteTxRx(self.port, ARM_JOINT_2, AX_TORUQE_ADDRESS, set_data)
        self.packet.write1ByteTxRx(self.port, ARM_TOOL, AX_TORUQE_ADDRESS, set_data)

    def set_joint(self, set_data):
        set_data = list(set_data)
        set_data[0] = np.int32(812 - set_data[0] / AX_TICK2DEG)
        set_data[1] = np.int32(212 + set_data[1] / AX_TICK2DEG)
        # print(set_data)
        self.packet.write2ByteTxRx(self.port, ARM_JOINT_1, AX_GOAL_POSITION, set_data[0])
        self.packet.write2ByteTxRx(self.port, ARM_JOINT_2, AX_GOAL_POSITION, set_data[1])

    def set_tool(self, set_data):
        set_data = np.int32(450 + set_data / AX_TICK2DEG)
        # print(set_data)
        self.packet.write2ByteTxRx(self.port, ARM_TOOL, AX_GOAL_POSITION, set_data)

    def get_arm(self):
        get_data = [0, 0, 0]
        get_data[0], result, err = self.packet.read2ByteTxRx(self.port, ARM_JOINT_1, AX_PRESENT_POSITION)
        get_data[1], result, err = self.packet.read2ByteTxRx(self.port, ARM_JOINT_2, AX_PRESENT_POSITION)
        get_data[2], result, err = self.packet.read2ByteTxRx(self.port, ARM_TOOL, AX_PRESENT_POSITION)

        get_data[0] = (812 - get_data[0]) * AX_TICK2RAD
        get_data[1] = (get_data[1] - 212) * AX_TICK2RAD
        get_data[2] = (get_data[2] - 450) * AX_TICK2RAD

        return get_data

if __name__ == '__main__':

    print('dynamixel arm controller')

    try:
        DXL_PORT.openPort()
        DXL_PORT.setBaudRate(BAUD_RATE)

        arm = Arm_Controller(DXL_PORT)
        arm.set_joint([45,45])
        arm.set_tool(45)
        time.sleep(2)
        while True:
            print(arm.get_arm())
            time.sleep(1)

    except KeyboardInterrupt:
        arm.reset()