#!/usr/bin/python3
#!/usr/bin/python2
# coding: utf-8
from i2clibraries import i2c_itg3205
from i2clibraries import i2c_adxl345
from i2clibraries import i2c_hmc5883l
from time import *
import math
import rospy
from riki_msgs.msg import Imu
from geometry_msgs.msg import Vector3, Quaternion


class BerryIMUPublisher:
    """Reads IMU data from BerryIMU using I2C bus and publishes data to ROS
    
    """
    def __init__(self):
        rospy.init_node('PY85', anonymous=False)
        self.pub = rospy.Publisher("/raw_imu", Imu, queue_size=1000)
        self.rate = rospy.Rate(int(rospy.get_param("~poll_rate", 20)))

        # Special setup for virtual I2C device so doesn't conflict with actual device
        is_virtual = int(rospy.get_param("~is_virtual", 1))
        gyr_addr = int(rospy.get_param("~gyr_addr", 0x68))
        mag_addr = int(rospy.get_param("~mag_addr", 0x1e))
        acc_addr = int(rospy.get_param("~acc_addr", 0x53))

        #if is_virtual:
            #self.IMU = IMU.BerryIMU(True, gyr_addr, mag_addr, acc_addr)
        #else:
            #self.IMU = IMU.BerryIMU()  # Initialise the accelerometer, gyroscope and compass

    def begin(self):
        """Keeps reading IMU data and publishing it to the topic imu_data
        
        """
        gyro_x_angle = 0.0
        gyro_y_angle = 0.0
        gyro_z_angle = 0.0
        last_yaw = 0
        first_run = True
        last_timestamp = rospy.get_time()
        adxl345 = i2c_adxl345.i2c_adxl345(1)
        itg3205 = i2c_itg3205.i2c_itg3205(1)
        (itgready, dataready) = itg3205.getInterruptStatus ()
        #hmc5883l = i2c_hmc5883l.i2c_hmc5883l(1)
        #hmc5883l.setContinuousMode ()
        #hmc5883l.setDeclination (2,4)
        while not rospy.is_shutdown():
            try:

                if dataready:
                  msg = Imu()
                  # Read the accelerometer,gyroscope and magnetometer values        
                  (gyr_x, gyr_y, gyr_z) = itg3205.getDegPerSecAxes ()        
                  (acc_x, acc_y, acc_z) = adxl345.getAxes ()        
                  #(mag_x, mag_y, mag_z) = hmc5883l.getAxes ()

                  # Calculate loop Period(LP). How long between Gyro Reads
                  period = (rospy.get_time() - last_timestamp)
                  last_timestamp = rospy.get_time()

                  # TODO: measure covariance
                  msg.linear_acceleration.x = acc_x * 1
                  msg.linear_acceleration.y = acc_y * 1
                  msg.linear_acceleration.z = acc_z * 1

                  msg.angular_velocity.x = gyr_x * 0.02
                  msg.angular_velocity.y = gyr_y * 0.02
                  msg.angular_velocity.z = gyr_z * 0.02

                  #msg.magnetic_field.x = mag_x 
                  #msg.magnetic_field.y = mag_y 
                  #msg.magnetic_field.z = mag_z

                  self.pub.publish(msg)
            except IOError as e:
                rospy.logwarn(e)
            self.rate.sleep()



if __name__ == "__main__":
    print('IMU Running!')
    try:
        berry = BerryIMUPublisher()
        berry.begin()
    except rospy.ROSInterruptException:
        pass




