#!/usr/bin/env python3

import rospy
from std_msgs.msg import Float32, Float32MultiArray, Bool, Empty

class PICK_AND_PLACE:
    def __init__(self):
        rospy.Subscriber('/pick_and_place', Bool, self.got_ball)

        self.arm_pub = rospy.Publisher("/arm", Float32MultiArray, queue_size=10)
        self.tool_pub = rospy.Publisher("/tool", Float32, queue_size=10)
        self.next_pub = rospy.Publisher('/next_path', Empty, queue_size=10)

    def got_ball(self, msg):
        # True : got
        # False : release
        arm = Float32MultiArray()
        arm.data.append(1.57)
        arm.data.append(-0.65)
        self.arm_pub.publish(arm)
        rospy.sleep(1)
        arm = Float32MultiArray()
        arm.data.append(0.7)
        arm.data.append(-0.65)
        self.arm_pub.publish(arm)
        rospy.sleep(2)
        tool = Float32()
        if msg.data:
            tool.data = -0.2
        else:
            tool.data = 0.71
        self.tool_pub.publish(tool)
        rospy.sleep(2)
        arm = Float32MultiArray()
        arm.data.append(1.57)
        arm.data.append(-0.65)
        self.arm_pub.publish(arm)
        rospy.sleep(1)
        arm = Float32MultiArray()
        arm.data.append(1.57)
        if msg.data:
            arm.data.append(-1.0)
        else:
            arm.data.append(-1.57)
        self.arm_pub.publish(arm)
        rospy.sleep(2)
        self.next_pub.publish()
        return True


if __name__ == '__main__':
    rospy.init_node('pick_and_place')
    node = PICK_AND_PLACE()

    rospy.spin()