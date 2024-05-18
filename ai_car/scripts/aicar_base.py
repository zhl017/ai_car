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

WHEEL_SEPERATION_X = 0.08
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

        self.arm = Arm_Controller(DXL_PORT)
        self.wheel = Wheel_Controller(DXL_PORT)

        rospy.Subscriber("/reset", Empty, self.cb_reset)
        rospy.Subscriber("/torque", Int8, self.cb_torque)
        rospy.Subscriber("/tool", Float32, self.cb_tool)
        rospy.Subscriber("/joint", Float32MultiArray, self.cb_joint)
        rospy.Subscriber("/cmd_vel", Twist, self.cb_cmd)

        joint_state_pub = rospy.Publisher("/joint_states", JointState, queue_size=10)
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

        rate = rospy.Rate(30)

        while not rospy.is_shutdown():

            self.fn_update_state()

            joint_state.header.stamp = rospy.Time.now()
            joint_state.position = [self.diff_left, self.diff_right, self.diff_rear_left, self.diff_rear_right,
                                    self.arm_state[0], self.arm_state[1], self.arm_state[2]]
            joint_state_pub.publish(joint_state)

            if self.tool_received: 
                self.arm.set_tool(self.tool_data)
                self.tool_received = False

            if self.joint_received:
                self.arm.set_joint(self.joint_data)
                self.joint_received = False

            if self.cmd_received:
                self.wheel.set_velocity(self.cmd_data)
                self.cmd_received = False

            if self.reset_received:
                self.arm.reset()
                self.wheel.reset()
                rospy.sleep(2)
                self.reset_received = False

            if self.torque_reveived:
                self.arm.set_torque(self.torque_data)
                self.torque_reveived = False

            rate.sleep()

        rospy.on_shutdown(self.fn_shutdown)

    def fn_update_state(self):
        self.arm_state = self.arm.get_arm()
        self.wheel_state = self.wheel.get_position()

        if not self.state_is_reset:
            self.last_left = self.wheel_state[0]
            self.last_right = self.wheel_state[1]
            self.last_rear_left = self.wheel_state[2]
            self.last_rear_right = self.wheel_state[3]
            self.state_is_reset = True

        current = self.wheel_state[0]
        self.diff_left = (current - self.last_left) * XM_TICK2RAD
        self.last_left = current
        if abs(self.diff_left) <= 2 * XM_TICK2RAD: self.diff_left = 0.0
        
        current = self.wheel_state[1]
        self.diff_right = (current - self.last_right) * XM_TICK2RAD
        self.last_right = current
        if abs(self.diff_right) <= 2 * XM_TICK2RAD: self.diff_right = 0.0

        current = self.wheel_state[2]
        self.diff_rear_left = (current - self.last_rear_left) * XM_TICK2RAD
        self.last_rear_left = current
        if abs(self.diff_rear_left) <= 2 * XM_TICK2RAD: self.diff_rear_left = 0.0

        current = self.wheel_state[3]
        self.diff_rear_right = (current - self.last_rear_right) * XM_TICK2RAD
        self.last_rear_right = current
        if abs(self.diff_rear_right) <= 2 * XM_TICK2RAD: self.diff_rear_right = 0.0

    def fn_shutdown(self):
        rospy.loginfo('Exiting ...')
        self.arm.reset()
        self.wheel.set_velocity([0,0])
        time.sleep(3)
        self.arm.set_torque(0)
        self.wheel.set_torque(0)
        DXL_PORT.closePort()

    def cb_reset(self, msg):
        rospy.loginfo('[ SET ] AICAR reset')
        self.state_is_reset = False
        self.reset_received = True

    def cb_torque(self, msg):
        rospy.loginfo('[ SET ] ARM torque : %d'%msg.data)
        self.torque_data = msg.data
        self.torque_reveived = True

    def cb_tool(self, msg):
        rospy.loginfo('[ SET ] ARM tool : %.2f'%msg.data)
        if not self.tool_received:
            self.tool_data = msg.data
            self.tool_received = True

    def cb_joint(self, msg):
        rospy.loginfo('[ SET ] ARM joint : %.2f, %.2f'%(msg.data[0],msg.data[1]))
        if not self.joint_received:
            self.joint_data = msg.data
            self.joint_received = True

    def cb_cmd(self, msg):
        if not self.cmd_received:
            lin_vel = msg.linear.x
            ang_vel = msg.angular.z

            left_vel = lin_vel - ang_vel * WHEEL_SEPERATION_X / 2
            right_vel = lin_vel + ang_vel * WHEEL_SEPERATION_X / 2

            left_vel = left_vel * VELOCITY2VALUE / 0.033
            right_vel = right_vel * VELOCITY2VALUE / 0.033

            left_vel = constrain(left_vel, -LIMIT_VEL, LIMIT_VEL)
            right_vel = constrain(right_vel, -LIMIT_VEL, LIMIT_VEL)

            rospy.loginfo('[ SET ] Wheel : %d, %d'%(left_vel, right_vel))
            self.cmd_data[0] = left_vel
            self.cmd_data[1] = right_vel
            self.cmd_received = True

if __name__ == "__main__":
    try:
        rospy.init_node('aicar_base')
        AICAR_BASE()

    except rospy.ROSInterruptException:
        pass