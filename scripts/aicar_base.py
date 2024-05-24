#!/usr/bin/env python3

import rospy
from std_msgs.msg import Int8, Empty, Float32, Float32MultiArray
from geometry_msgs.msg import Twist
from sensor_msgs.msg import JointState

from dynamixel_sdk import *
from ai_car.dynamixel_arm import Arm_Controller
from ai_car.dynamixel_wheel import Wheel_Controller

BAUD_RATE = 1000000
DXL_PORT = PortHandler('/dev/U2D2')

WHEEL_SEPERATION = 0.08
LIMIT_VEL = 200

VELOCITY2VALUE = 41.69988758
XM_TICK2RAD = 0.001533981

def constrain(vel, min_vel, max_vel):
    if vel < min_vel: return min_vel
    if vel > max_vel: return max_vel
    return vel

class AICAR_BASE:
    def __init__(self):
        rospy.loginfo('Connect U2D2 ...')
        try:
            DXL_PORT.setBaudRate(BAUD_RATE)
            rospy.loginfo('[ OK ] AICAR start')
        except:
            rospy.loginfo('[ FAIL ] U2D2 not found !')
            exit()

        # create arm & wheel controller class
        self.arm = Arm_Controller(DXL_PORT)
        self.wheel = Wheel_Controller(DXL_PORT)

        """

        try to setup pub & sub
     
        """

        # create Jointstate message
        joint_state = JointState()
        joint_state.name = ['wheel_left_joint','wheel_right_joint','wheel_rear_left_joint', 'wheel_rear_right_joint',
                            'arm_joint_1', 'arm_joint_2', 'tool_joint']

        self.torque_data = 0
        self.tool_data = 0.0
        self.joint_data = [0.0, 0.0]
        self.cmd_data = [0.0, 0.0]

        self.reset_received = False
        self.torque_reveived = False
        self.tool_received = False
        self.joint_received = False
        self.cmd_received = False
        self.state_is_reset = False
        self.last_left = self.last_right = self.last_rear_left = self.last_rear_right = 0
        self.diff_left = self.diff_right = self.diff_rear_left = self.diff_rear_right = 0
        self.total_left = self.total_right = self.total_rear_left = self.total_rear_right = 0

        rate = rospy.Rate(30)

        while not rospy.is_shutdown():

            self.fn_update_state()

            """
            
            try to setup data pub & sub
            
            """

            if self.tool_received: 
                self.tool_received = False

            if self.joint_received:
                self.joint_received = False

            if self.cmd_received:
                self.cmd_received = False

            if self.reset_received:
                self.reset_received = False

            if self.torque_reveived:
                self.torque_reveived = False

            rate.sleep()

        rospy.on_shutdown(self.fn_shutdown)

    def fn_update_state(self):
        pass

    def fn_shutdown(self):
        rospy.loginfo('Exiting ...')
        DXL_PORT.closePort()

    def cb_reset(self, msg):
        rospy.loginfo('[ SET ] AICAR reset')
        self.total_left = self.total_right = self.total_rear_left = self.total_rear_right = 0
        self.state_is_reset = False
        self.reset_received = True

    def cb_torque(self, msg):
        rospy.loginfo('[ SET ] ARM torque : %d'%msg.data)
        self.torque_data = msg.data
        self.torque_reveived = True

    def cb_tool(self, msg):
        rospy.loginfo('[ SET ] ARM tool : %.2f'%msg.data)
        if not self.tool_received:
            self.tool_received = True

    def cb_joint(self, msg):
        rospy.loginfo('[ SET ] ARM joint : %.2f, %.2f'%(msg.data[0],msg.data[1]))
        if not self.joint_received:
            self.joint_received = True

    def cb_cmd(self, msg):
        if not self.cmd_received:
            rospy.loginfo('[ SET ] Wheel : %d, %d'%(left_vel, right_vel))
            self.cmd_received = True

if __name__ == "__main__":
    try:
        rospy.init_node('aicar_base')
        AICAR_BASE()

    except rospy.ROSInterruptException:
        pass
