#!/usr/bin/env python3

import rospy, cv2
import numpy as np
from sensor_msgs.msg import Image
from std_msgs.msg import Float32, Float32MultiArray, Bool, Empty
from geometry_msgs.msg import Twist
from cv_bridge import CvBridge, CvBridgeError

class CV_DETECTOR:
    def __init__(self):
        rospy.Subscriber('/camera/image', Image, self.cb_image)
        rospy.Subscriber('/cv_detect', Empty, self.cb_reset)

        self.image_pub = rospy.Publisher("/ball_detect", Image, queue_size=10)
        self.pap_pub = rospy.Publisher('/pick_and_place', Bool, queue_size=10)
        self.cmd_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)

        self.bridge = CvBridge()
        self.img_org = None
        self.saw_image = False

    def cb_reset (self, msg):
        self.saw_image = True

    def cb_image(self, msg):
        if self.saw_image:
            try:
                cv_image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
            except CvBridgeError as e:
                rospy.logerr(e)
                return

            gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
            gray_blurred = cv2.GaussianBlur(gray, (5, 5), 2)

            circles = cv2.HoughCircles(gray_blurred, cv2.HOUGH_GRADIENT_ALT, dp=1.5, minDist=200,
                                    param1=100, param2=0.9, minRadius=8, maxRadius=0)

            if circles is not None:
                circles = np.uint16(np.around(circles))
                for i in circles[0, :]:
                    cv2.circle(cv_image, (i[0], i[1]), i[2], (0, 255, 0), 2)
                    cv2.circle(cv_image, (i[0], i[1]), 2, (0, 0, 255), 3)

                    rospy.loginfo("Circle detected at x: %d, y: %d" % (i[0], i[1]))
                    rospy.loginfo("Circle radius: %f" % i[2])

                    cmd = Twist()
                    cnt = 0
                    if i[2] < 18:
                        cmd.linear.x = 0.01
                    elif i[2] > 19:
                        cmd.linear.x = -0.01
                    else:
                        cmd.linear.x = 0
                        cnt += 1

                    if i[0] < 160:
                        cmd.angular.z = 0.02
                    elif i[0] > 160:
                        cmd.angular.z = -0.02
                    else:
                        cmd.angular.z = 0
                        cnt += 1
                        
                    self.cmd_pub.publish(cmd)

                    if cnt == 2:
                        rospy.loginfo('Got')
                        rospy.sleep(1)
                        cmd = Twist()
                        cmd.linear.x = 0
                        cmd.angular.z = 0
                        self.cmd_pub.publish(cmd)
                        self.pap_pub.publish(True)
                        self.saw_image = False
                    
            try:
                self.image_pub.publish(self.bridge.cv2_to_imgmsg(cv_image, "bgr8"))
            except CvBridgeError as e:
                rospy.logerr(e)

if __name__ == '__main__':
    rospy.init_node('cv_detector')
    cv = CV_DETECTOR()

    rospy.spin()