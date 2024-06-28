
import rospy, RTIMU
from sensor_msgs.msg import Imu
from std_msgs.msg import Empty

class MPU9250:
    def __init__(self):
        rospy.Subscriber("/reset", Empty, self.cb_reset)

        imu_pub = rospy.Publisher("/imu", Imu, queue_size=10)
        imu_data = Imu()

        rate = rospy.Rate(200)

        s = RTIMU.Settings("/home/ubuntu/catkin_ws/src/ai_car/scripts/RTIMULib")
        self.imu = RTIMU.RTIMU(s)
        rospy.loginfo("IMU Name: " + self.imu.IMUName())
        if (not self.imu.IMUInit()):
            rospy.loginfo("IMU Init Failed")
        else:
            rospy.loginfo("IMU Init Succeeded")

        self.imu.setSlerpPower(0.02)
        self.imu.setGyroEnable(True)
        self.imu.setAccelEnable(True)
        self.imu.setCompassEnable(True)

        self.last_ang = 0

        while not rospy.is_shutdown():
            if self.imu.IMURead():
                data = self.imu.getIMUData()
                Pose = data["fusionPose"]
                QPose = data["fusionQPose"]
                gyro = data["gyro"]
                accel = data["accel"]

                imu_data.header.stamp = rospy.Time.now()
                imu_data.header.frame_id = "imu"
                
                ang = gyro[2]
                if abs(ang) >= 2.0: ang = 0.0
                imu_data.angular_velocity.z = -ang

                imu_pub.publish(imu_data)

                self.last_ang = ang
                rate.sleep()

    def cb_reset(self, msg):
        rospy.loginfo('[ SET ] MPU9250 reset')
        if (not self.imu.IMUInit()):
            rospy.loginfo("IMU Init Failed")
        else:
            rospy.loginfo("IMU Init Succeeded")

        self.imu.setSlerpPower(0.02)
        self.imu.setGyroEnable(True)
        self.imu.setAccelEnable(True)
        self.imu.setCompassEnable(True)

if __name__ == '__main__':          
    try:      
        rospy.init_node('aicar_imu')
        mpu9250 = MPU9250()

    except rospy.ROSInterruptException:
        pass