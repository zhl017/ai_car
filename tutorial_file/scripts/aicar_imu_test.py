
import rospy, RTIMU
from sensor_msgs.msg import Imu
from std_msgs.msg import Empty

class MPU9250:
    def __init__(self):

        """

        try to setup pub & sub
     
        """

        # create imu message
        imu_data = Imu()

        rate = rospy.Rate(200)

        s = RTIMU.Settings("/home/ubuntu/catkin_ws/src/ai_car_tutorial/scripts/RTIMULib")
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

        while not rospy.is_shutdown():
            if self.imu.IMURead():
                # mpu9250 get imudata
                data = self.imu.getIMUData()
                Pose = data["fusionPose"]
                QPose = data["fusionQPose"]
                gyro = data["gyro"]
                accel = data["accel"]

                """

                try to pub imu data

                """

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