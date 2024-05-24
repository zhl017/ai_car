#!/usr/bin/env python3

import rospy, tf
from math import sin, cos
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Point, Quaternion, Twist
from std_msgs.msg import Empty, Float32MultiArray
from tf.transformations import quaternion_from_euler

XM_TICK2RAD = 0.001533981
WHEEL_RADIUS = 0.033
WHEEL_SEPARATION = 0.08

class AICAR_ODOM:
    def __init__(self):

        """
        
        try to setup pub & sub

        """

        # create odom message
        self.odom = Odometry()
        self.odom.header.frame_id = 'odom'
        self.odom.child_frame_id = 'base_footprint'

        # create tf boardcaster
        self.boardcaster = tf.TransformBroadcaster()

        self.last_time = rospy.Time.now()
        self.imu_is_reset = False
        self.x = self.y = self.theta = 0
        self.imu_ang = 0

        rospy.spin()

    def cb_reset(self, msg):
        rospy.loginfo('[ SET ] Odom reset')
        self.imu_is_reset = False
        self.x = self.y = self.theta = 0
        self.imu_ang = 0
        rospy.sleep(2)

    def cb_calc_odom(self, msg):
        current_time = rospy.Time.now()
        dt = (current_time - self.last_time).to_sec()

        """

        try calc odom information
        
        """

        # publish odom topic data
        self.odom.header.stamp = current_time
        self.odom.pose.pose.position = Point(self.x, self.y, 0)
        self.odom.pose.pose.orientation = Quaternion(0, 0, sin(self.theta/2), cos(self.theta/2))
        self.odom.twist.twist = Twist(Point(lin, 0, 0), Point(0, 0, ang))
        self.odom_pub.publish(self.odom)

        # tf transform odom frame > base_footprint frame
        odom_quat = quaternion_from_euler(0, 0, self.theta)
        self.boardcaster.sendTransform(
            (self.x, self.y, 0),
            odom_quat,
            current_time,
            'base_footprint',
            'odom'
        )

        self.last_time = current_time

if __name__ == "__main__":
    try:
        rospy.init_node('aicar_odom')
        AICAR_ODOM()

    except rospy.ROSInterruptException:
        pass