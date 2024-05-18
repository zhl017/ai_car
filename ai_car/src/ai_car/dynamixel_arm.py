#!/usr/bin/env python3

import numpy as np
from dynamixel_sdk import *

BAUD_RATE = 1000000
DXL_PORT = PortHandler('/dev/ttyUSB0')

AX_LED_ADDRESS = 25
AX_TORUQE_ADDRESS = 24
AX_GOAL_POSITION = 30
AX_MOVING_SPEED = 32
AX_PRESENT_POSITION = 36

ARM_JOINT_1 = 11            # 812 ~ 512     0.0 ~ 1.57
ARM_JOINT_2 = 12            # 212 ~ 512     -1.57 ~ 0.0
ARM_TOOL = 13               # 420 ~ 670     -0.47 ~ 0.79

HOME_POSE = [1.57, -1.57]   # 812, 212
HOME_TOOl = 0.80

AX_TICK2RAD = 0.005113269      # 300 / 1024 / 180 * pi

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
        self.set_joint(HOME_POSE)
        self.set_tool(HOME_TOOl)

    def set_torque(self, set_data):
        set_data = np.int32(set_data)
        self.packet.write1ByteTxRx(self.port, ARM_JOINT_1, AX_TORUQE_ADDRESS, set_data)
        self.packet.write1ByteTxRx(self.port, ARM_JOINT_2, AX_TORUQE_ADDRESS, set_data)
        self.packet.write1ByteTxRx(self.port, ARM_TOOL, AX_TORUQE_ADDRESS, set_data)

    def set_led(self, set_data):
        set_data = np.int32(set_data)
        self.packet.write1ByteTxRx(self.port, ARM_JOINT_1, AX_LED_ADDRESS, set_data)
        self.packet.write1ByteTxRx(self.port, ARM_JOINT_2, AX_LED_ADDRESS, set_data)
        self.packet.write1ByteTxRx(self.port, ARM_TOOL, AX_LED_ADDRESS, set_data)

    def set_joint(self, set_data):
        set_data = list(set_data)
        set_data[0] = np.int32((set_data[0] + 2.62) / AX_TICK2RAD)
        set_data[1] = np.int32((set_data[1] + 2.62) / AX_TICK2RAD)
        # print(set_data)
        self.packet.write2ByteTxRx(self.port, ARM_JOINT_1, AX_GOAL_POSITION, set_data[0])
        self.packet.write2ByteTxRx(self.port, ARM_JOINT_2, AX_GOAL_POSITION, set_data[1])

    def set_tool(self, set_data):
        set_data = np.int32((set_data + 2.62) / AX_TICK2RAD)
        # print(set_data)
        self.packet.write2ByteTxRx(self.port, ARM_TOOL, AX_GOAL_POSITION, set_data)

    def get_arm(self):
        get_data = [0, 0, 0]
        get_data[0], result, err = self.packet.read2ByteTxRx(self.port, ARM_JOINT_1, AX_PRESENT_POSITION)
        get_data[1], result, err = self.packet.read2ByteTxRx(self.port, ARM_JOINT_2, AX_PRESENT_POSITION)
        get_data[2], result, err = self.packet.read2ByteTxRx(self.port, ARM_TOOL, AX_PRESENT_POSITION)

        get_data[0] = (get_data[0] - 512) * AX_TICK2RAD
        get_data[1] = (get_data[1] - 512) * AX_TICK2RAD
        get_data[2] = (get_data[2] - 512) * AX_TICK2RAD

        return get_data

if __name__ == '__main__':

    print('dynamixel arm controller')
    DXL_PORT.openPort()
    DXL_PORT.setBaudRate(BAUD_RATE)

    arm = Arm_Controller(DXL_PORT)

    arm.set_led(0)
    time.sleep(1)

    arm.set_led(1)
    time.sleep(1)

    arm.set_joint([0, 0])
    arm.set_tool(-0.5)
    time.sleep(2)

    arm.set_joint([0.7, -0.7])
    arm.set_tool(0.5)
    time.sleep(2)

    arm.reset()