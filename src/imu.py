#!/usr/bin/python3
# coding: utf-8

import time
import rospy
from sensor_msgs.msg import Imu, MagneticField

from i2clibraries import i2c_itg3205
from i2clibraries import i2c_adxl345
from i2clibraries import i2c_hmc5883l
from threading import Thread
itg3205 = i2c_itg3205.i2c_itg3205(1)
adxl345 = i2c_adxl345.i2c_adxl345(1)
hmc5883l = i2c_hmc5883l.i2c_hmc5883l(1)
hmc5883l.setContinuousMode
hmc5883l.setDeclination(9,54)

def init():
	
	(x, y, z) = itg3205.getDegPerSecAxes()
	#print("Temp: "+str(temp))
	#print("ITG X:    "+str(x))
	#print("ITG Y:    "+str(y))
	#print("ITG Z:    "+str(z))
	(x2, y2, z2) = adxl345.getAxes()
	#print(x2)
	#print(y2)
	#print(z2)
	(x3, y3, z3) = hmc5883l.getAxes()
	#print(x3)
	#print(y3)
	#print(z3)
	#time.sleep(1)
	return x, y, z, x2, y2, z2, x3, y3, z3

if __name__ == '__main__':	
	rospy.init_node('imu_node')
	imu_pub = rospy.Publisher('/raw_imu', Imu, queue_size=10)
	mag_pub = rospy.Publisher('mag', MagneticField, queue_size=10)
	print('IMU working!')
	while not rospy.is_shutdown():
		(x, y, z, x2, y2, z2, x3, y3, z3) = init()
		imu = Imu()
		imu.linear_acceleration.x = x
		imu.linear_acceleration.y = y
		imu.linear_acceleration.z = z

		imu.angular_velocity.x = x2
		imu.angular_velocity.y = y2
		imu.angular_velocity.z = z2

		imu.orientation.x = 0
		imu.orientation.y = 0
		imu.orientation.z = 0
		imu.orientation.w = 0

		imu_pub.publish(imu)

		#mag = MagneticField()
		#mag.magnetic_field.x = x3
		#mag.magnetic_field.y = y3
		#mag.magnetic_field.z = z3
		#mag_pub.publish(mag)
	rospy.spin














