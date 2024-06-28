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

        rospy.wait_for_message("/imu", Imu)
        rospy.Subscriber("/diff_rad", Float32MultiArray, self.cb_calc_odom)
        rospy.Subscriber("/imu", Imu, self.cb_imu)
        rospy.Subscriber("/reset", Empty, self.cb_reset)
        
        self.odom_pub = rospy.Publisher("/odom", Odometry, queue_size=10)
        self.odom = Odometry()
        self.odom.header.frame_id = 'odom'
        self.odom.child_frame_id = 'base_footprint'

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

    def cb_imu(self, msg):
        self.imu_ang = msg.angular_velocity.z
        self.imu_is_reset = True

    def cb_calc_odom(self, msg):
        if self.imu_is_reset:
            if msg.data[0] == 0 and msg.data[1] == 0:
                ang = 0.0
            else:
                ang = self.imu_ang
            
            current_time = rospy.Time.now()

            dt = (current_time - self.last_time).to_sec()

            # calc l r distance
            l_distance = msg.data[0] * WHEEL_RADIUS
            r_distance = msg.data[1] * WHEEL_RADIUS

            # calc linear & angular
            lin = (l_distance + r_distance) / (2.0 * dt)
            # ang = (r_distance - l_distance) / (WHEEL_SEPARATION * dt)

            # calc mov x, y
            delta_x = lin * cos(self.theta) * dt
            delta_y = lin * sin(self.theta) * dt
            delta_theta = ang * dt

            self.x += delta_x
            self.y += delta_y
            self.theta += delta_theta

            self.odom.header.stamp = current_time
            self.odom.pose.pose.position = Point(self.x, self.y, 0)
            self.odom.pose.pose.orientation = Quaternion(0, 0, sin(self.theta/2), cos(self.theta/2))
            self.odom.twist.twist = Twist(Point(lin, 0, 0), Point(0, 0, ang))

            self.odom_pub.publish(self.odom)

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

