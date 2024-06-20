#!/usr/bin/env python3

import rospy, roslaunch, actionlib
from actionlib_msgs.msg import *
from geometry_msgs.msg import Twist, Pose, PoseWithCovarianceStamped, Quaternion, Point
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from nav_msgs.msg import Odometry
from std_msgs.msg import Empty, Bool
from random import sample  
from math import pow, sqrt  

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
        self.pnp = rospy.Publisher('pick_and_place', Bool, queue_size=10)

        # Action
        self.move_base = actionlib.SimpleActionClient('/move_base', MoveBaseAction)

        # Param
        self.is_initialed = False

        self.rest_time = rospy.get_param("~rest_time", 1)

        goal_states = ['PENDING', 'ACTIVE', 'PREEMPTED','SUCCEEDED',  
                       'ABORTED', 'REJECTED','PREEMPTING', 'RECALLING',   
                       'RECALLED','LOST']  
         
        locations = dict()
        locations['1'] = Pose(Point(1.7, 0, 0.00), Quaternion(0.000, 0.000, 0.000, 1.000))  
        locations['2'] = Pose(Point(1.7, 0, 0.00), Quaternion(0.000, 0.000, -0.99, 0.000))

        # First time to initial pose
        rospy.sleep(3)
        self.fnInit()
        while True:
            if self.is_initialed == True:
                break
        rospy.loginfo('DONE')
        rospy.sleep(5)

        self.path_1 = True
        self.pick = False
        self.path_2 = False
        self.place = False
  
        # Begin the main loop and run through a sequence of locations  
        while not rospy.is_shutdown(): 

            if self.path_1:
                # Set up the next goal location  
                self.goal = MoveBaseGoal()    
                self.goal.target_pose.header.frame_id = 'map'  
                self.goal.target_pose.header.stamp = rospy.Time.now() 
                self.goal.target_pose.pose = locations['1'] 

                # Let the user know where the robot is going next  
                rospy.loginfo("Going to: path 1")  
                # Start the robot toward the next location  
                self.move_base.send_goal(self.goal)  

                # Allow 5 minutes to get there  
                finished_within_time = self.move_base.wait_for_result(rospy.Duration(300))  

                # Check for success or failure  
                if not finished_within_time:  
                    self.move_base.cancel_goal()  
                    rospy.loginfo("Timed out achieving goal")  
                else:  
                    state = self.move_base.get_state()  
                    if state == GoalStatus.SUCCEEDED:  
                        rospy.loginfo("Goal succeeded!")
                        self.path_1 = False
                        self.pick = True
                        self.detect_pub.publish()
                    else:  
                        rospy.loginfo("Goal failed with error code: " + str(goal_states[state]))  
                rospy.sleep(self.rest_time)

            if self.path_2:
                # Set up the next goal location  
                self.goal = MoveBaseGoal()    
                self.goal.target_pose.header.frame_id = 'map'  
                self.goal.target_pose.header.stamp = rospy.Time.now() 
                self.goal.target_pose.pose = locations['2'] 

                # Let the user know where the robot is going next  
                rospy.loginfo("Going to: path 2")  
                # Start the robot toward the next location  
                self.move_base.send_goal(self.goal)  

                # Allow 5 minutes to get there  
                finished_within_time = self.move_base.wait_for_result(rospy.Duration(300))  

                # Check for success or failure  
                if not finished_within_time:  
                    self.move_base.cancel_goal()  
                    rospy.loginfo("Timed out achieving goal")  
                else:  
                    state = self.move_base.get_state()  
                    if state == GoalStatus.SUCCEEDED:  
                        rospy.loginfo("Goal succeeded!")
                        self.path_2 = False
                        self.place = True
                        self.pnp.publish(False)
                    else:  
                        rospy.loginfo("Goal failed with error code: " + str(goal_states[state]))  
                rospy.sleep(self.rest_time)

    def cbNext(self, msg):
        if self.pick:
            self.path_2 = True
            self.pick = False
        if self.place:
            self.path_1 = True
            self.place = False


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
        initialPose.pose.pose.position.x = 1.0
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