#!/usr/bin/env python3

import rospy, sys, select, termios, tty
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32, Float32MultiArray
from sensor_msgs.msg import JointState

MAX_LIN_VEL = 0.16
MAX_ANG_VEL = 4.0

STEP_LIN_VEL = 0.01
STEP_ANG_VEL = 1.0

MAX_J1_RAD = 1.57
MIN_J1_RAD = 0.0

MAX_J2_RAD = 0.0
MIN_J2_RAD = -1.57

MAX_TOOL_RAD = 0.79
MIN_TOOL_RAD = -0.47

STEP_ARM = 0.03

msg = """
Control Your AICAR!
----------------------------------------------
Moving around:
            w(+)                   
   a(+)      s      d(-)             
            x(-) 

Arm control:
    Joint1 : u(-), i(+)
    Joint2 : j(+), k(-)
     Tool  : m(-), ,(+)

h : Arm Home Pose
s : force stop

CTRL-C to quit
"""

def constrain(vel, min_vel, max_vel):
    if vel < min_vel: return min_vel
    if vel > max_vel: return max_vel
    return vel

class AICAR_TELEOP:
    def __init__(self):

        rospy.Subscriber("/joint_states", JointState, self.cb_joint_states)

        self.cmd_pub = rospy.Publisher("/cmd_vel", Twist, queue_size=10)
        self.joint_pub = rospy.Publisher("/joint", Float32MultiArray, queue_size=10)
        self.tool_pub = rospy.Publisher("/tool", Float32, queue_size=10)

        self.settings = termios.tcgetattr(sys.stdin)

        self.lin_vel = 0.0
        self.ang_vel = 0.0

        self.get_arm_data = False

        status = 0
        
        while not rospy.is_shutdown():
            key = self.getKey()

            if key == 'w':
                self.lin_vel = constrain((self.lin_vel + STEP_LIN_VEL), -MAX_LIN_VEL, MAX_LIN_VEL)
                self.wheel_info()
                status += 1

            elif key == 'x':
                self.lin_vel = constrain((self.lin_vel - STEP_LIN_VEL), -MAX_LIN_VEL, MAX_LIN_VEL)
                self.wheel_info()
                status += 1

            elif key == 'a':
                self.ang_vel = constrain((self.ang_vel + STEP_ANG_VEL), -MAX_ANG_VEL, MAX_ANG_VEL)
                self.wheel_info()
                status += 1

            elif key == 'd':
                self.ang_vel = constrain((self.ang_vel - STEP_ANG_VEL), -MAX_ANG_VEL, MAX_ANG_VEL)
                self.wheel_info()
                status += 1

            elif key == 's':
                self.lin_vel = 0.0
                self.ang_vel = 0.0
                self.wheel_info()
                status += 1

            elif key == 'u':
                self.j1_rad = constrain((self.j1_rad - STEP_ARM), MIN_J1_RAD, MAX_J1_RAD)
                self.arm_info()
                status += 1

            elif key == 'i':
                self.j1_rad = constrain((self.j1_rad + STEP_ARM), MIN_J1_RAD, MAX_J1_RAD)
                self.arm_info()
                status += 1
                
            elif key == 'j':
                self.j2_rad = constrain((self.j2_rad + STEP_ARM), MIN_J2_RAD, MAX_J2_RAD)
                self.arm_info()
                status += 1
                
            elif key == 'k':
                self.j2_rad = constrain((self.j2_rad - STEP_ARM), MIN_J2_RAD, MAX_J2_RAD)
                self.arm_info()
                status += 1
                
            elif key == 'm':
                self.tool_rad = constrain((self.tool_rad - STEP_ARM), MIN_TOOL_RAD, MAX_TOOL_RAD)
                self.arm_info()
                status += 1
                
            elif key == ',':
                self.tool_rad = constrain((self.tool_rad + STEP_ARM), MIN_TOOL_RAD, MAX_TOOL_RAD)
                self.arm_info()
                status += 1

            elif key == 'h':
                self.j1_rad = 1.57
                self.j2_rad = -1.57
                self.tool_rad = 0.79
                self.arm_info()
                status += 1

            else:
                if (key == '\x03'):
                    break

            if status == 20:
                print(msg)
                status = 0

    def getKey(self):
        tty.setraw(sys.stdin.fileno())
        select.select([sys.stdin], [], [], 0)
        key = sys.stdin.read(1)
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)
        return key
    
    def cb_joint_states(self, msg):
        if not self.get_arm_data:
            self.j1_rad = msg.position[4]
            self.j2_rad = msg.position[5]
            self.tool_rad = msg.position[6]
            self.get_arm_data = True
        else:
            return
    
    def wheel_info(self):
        print('[ WHEEL ] lineagr : %.2f, angular : %.2f'%(self.lin_vel, self.ang_vel))
        twist = Twist()
        twist.linear.x = self.lin_vel
        twist.angular.z = self.ang_vel
        self.cmd_pub.publish(twist)

    def arm_info(self):
        print('[  ARM  ] j1 : %.2f, j2 : %.2f, tool : %.2f'%(self.j1_rad, self.j2_rad, self.tool_rad))
        joint = Float32MultiArray()
        joint.data.append(self.j1_rad)
        joint.data.append(self.j2_rad)
        self.joint_pub.publish(joint)
        tool = Float32()
        tool.data = self.tool_rad
        self.tool_pub.publish(tool)


if __name__ == '__main__':
    try:
        rospy.init_node('aicar_teleop')
        print(msg)
        AICAR_TELEOP()
        
    except rospy.ROSInterruptException:
        pass
