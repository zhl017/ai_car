#!/usr/bin/env python3

import rospy, roslaunch, actionlib
from actionlib_msgs.msg import *
from geometry_msgs.msg import Twist, PoseWithCovarianceStamped
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from nav_msgs.msg import Odometry
from std_msgs.msg import Empty, Bool
from enum import Enum

class AutoGoal():
    def __init__(self):
        rospy.on_shutdown(self.fnShutdown)  

        # Launch navigation file
        self.uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
        self.ros_package_path = '/home/zhl017/catkin_ws/src/ai_car/launch/nav.launch'     
        self.fnLaunch()

        # Subscriber
        self.sub_odom = rospy.Subscriber('/odom', Odometry, self.cbOdom, queue_size=1)
        rospy.Subscriber('/next_path', Empty, self.cbNext)
        
        # Publisher
        self.pub_cmd_vel = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
        self.pub_init_pose = rospy.Publisher('/initialpose', PoseWithCovarianceStamped, queue_size=1)
        self.detect_pub = rospy.Publisher('/cv_detect', Empty, queue_size=10)
        self.pnp = rospy.Publisher('/pick_and_place', Bool, queue_size=10)

        # Action
        self.move_base = actionlib.SimpleActionClient('/move_base', MoveBaseAction)
        self.move_base.wait_for_server()

        # Param
        self.is_initialed = False

        self.rest_time = rospy.get_param("~rest_time", 1)

        # First time to initial pose
        rospy.sleep(3)
        self.fnInit()
        while True:
            if self.is_initialed == True:
                break
        rospy.loginfo('DONE')
        rospy.sleep(3)

        # self.p1 = True
        # self.pick = False
        # self.p2 = False
        # self.place = False
        # self.p3 = False
        
        self.path = Enum('path', 'idle p1 p2 p3')
        self.current_path = self.path.p1.value
        path1 = [2.5, 0.0, 0.0, 1.0]
        path2 = [2.5, 0.0, -0.7, 0.7]
        path3 = [2.5, -1.0, 1.0, 0.0]
        self.pick = False
        self.place = False
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = 'map'
  
        # Begin the main loop and run through a sequence of locations  
        while not rospy.is_shutdown(): 
            
            if self.current_path == self.path.p1.value:
                p = path1
            elif self.current_path == self.path.p2.value:
                p = path2
            elif self.current_path == self.path.p3.value:
                p = path3
            else:
                p = None

            rospy.loginfo(self.current_path)
            rospy.loginfo(p)

            if p != None:
                goal.target_pose.header.stamp = rospy.Time.now()
                goal.target_pose.pose.position.x = p[0]
                goal.target_pose.pose.position.y = p[1]
                goal.target_pose.pose.orientation.z = p[2]
                goal.target_pose.pose.orientation.w= p[3]

                self.move_base.send_goal(goal)
                self.move_base.wait_for_result()

                if self.current_path == self.path.p1.value:
                    self.current_path = self.path.idle.value
                    self.pick = True
                    self.detect_pub.publish()
                elif self.current_path == self.path.p2.value:
                    self.current_path = self.path.p3.value
                elif self.current_path == self.path.p3.value:
                    self.current_path = self.path.idle.value
                    self.place = True
                    self.pnp.publish(False)

            # if self.p1:
            #     self.p1 = False
            #     goal = MoveBaseGoal()
            #     goal.target_pose.header.frame_id = 'map'
            #     goal.target_pose.header.stamp = rospy.Time.now()
            #     goal.target_pose.pose.position.x = 0.9
            #     goal.target_pose.pose.position.y = 0.0
            #     goal.target_pose.pose.orientation.w = 1.0
            #     self.move_base.send_goal(goal)
            #     self.move_base.wait_for_result()
            #     rospy.loginfo("path 1 GOAL")
            #     self.pick = True
            #     self.detect_pub.publish()
            # if self.p2:
            #     self.p2 = False
            #     goal = MoveBaseGoal()
            #     goal.target_pose.header.frame_id = 'map'
            #     goal.target_pose.header.stamp = rospy.Time.now()
            #     goal.target_pose.pose.position.x = 0.9
            #     goal.target_pose.pose.position.y = 0.0
            #     goal.target_pose.pose.orientation.z = 0.7
            #     goal.target_pose.pose.orientation.w = 0.7
            #     self.move_base.send_goal(goal)
            #     self.move_base.wait_for_result()
            #     rospy.loginfo("path 2 GOAL")
            #     self.p3 = True
            # if self.p3:
            #     self.p3 = False
            #     goal = MoveBaseGoal()
            #     goal.target_pose.header.frame_id = 'map'
            #     goal.target_pose.header.stamp = rospy.Time.now()
            #     goal.target_pose.pose.position.x = 0.7
            #     goal.target_pose.pose.position.y = 1.8
            #     goal.target_pose.pose.orientation.z = -0.7
            #     goal.target_pose.pose.orientation.w = -0.7
            #     self.move_base.send_goal(goal)
            #     self.move_base.wait_for_result()
            #     rospy.loginfo("path 3 GOAL")
            #     self.place = True
            #     self.pnp.publish(False)

    def cbNext(self, msg):
        if self.pick:
            self.pick = False
            self.current_path = self.path.p2.value
        if self.place:
            self.place = False
            self.current_path = self.path.p1.value


    def fnLaunch(self):
        launcher = roslaunch.scriptapi.ROSLaunch()
        launcher = roslaunch.parent.ROSLaunchParent(self.uuid, [self.ros_package_path])
        launcher.start()

    def fnInit(self):
        rospy.loginfo('SETTING INITIAL POSE ...')
        initialPose = PoseWithCovarianceStamped()
        initialPose.header.frame_id = 'map'
        initialPose.header.stamp = rospy.Time.now()
        # initialPose.pose.pose = self.odom_msg.pose.pose
        initialPose.pose.pose.position.x = 0.0
        initialPose.pose.pose.position.y = 0.0
        initialPose.pose.pose.position.z = 0.0
        initialPose.pose.pose.orientation.x = 0.0
        initialPose.pose.pose.orientation.y = 0.0
        initialPose.pose.pose.orientation.z = 0.0
        initialPose.pose.pose.orientation.w = 1.0
        self.pub_init_pose.publish(initialPose)
        self.is_initialed = True

    def fnShutdown(self):
        rospy.loginfo("STOPPING THE ROBOT ...")
        self.move_base.cancel_goal()
        self.pub_cmd_vel.publish(Twist())
        rospy.sleep(1)

    def cbOdom(self, msg):
        self.odom_msg = msg

    def main(self):
        rospy.spin()

if __name__ == '__main__':
    rospy.init_node('turtlebot3_multinav')
    node = AutoGoal()
    node.main()