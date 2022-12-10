#! /usr/bin/python3

# Based on https://github.com/ControlEverythingCommunity/L3G4200D

import rospy
import numpy as np
import smbus
import time
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Quaternion
from tf.transformations import quaternion_multiply

# publish gyro readings from a L3G4200D
def gyro():
    #initialise node
    rospy.init_node("l3g4200d")
    # create publisher
    pub = rospy.Publisher("gyro", Imu, queue_size=1)
    # define publish rate
    f = 50
    dt = 1/f
    rate = rospy.Rate(50)
    
    # Get I2C bus
    bus = smbus.SMBus(1)

    # L3G4200D address, 0x69(105)
    # Select Control register1, 0x20(32)
    #		0x0F(15)	Normal mode, X, Y, Z-Axis enabled
    bus.write_byte_data(0x69, 0x20, 0x0F)

    # L3G4200D address, 0x69(105)
    # Select Control register4, 0x23(35)
    #                               Continous update, Data LSB at lower address
    #           0x00(0)                 FSR 250dps, dps per digit 0.00875, 
    #                                       Self test disabled, 4-wire interface
    #           0x10(16)                FSR 500dps, dps per digit 0.01750, 
    #                                       Self test disabled, 4-wire interface
    #		0x30(48)	        FSR 2000dps, dps per digit 0.07,
    #                                       Self test disabled, 4-wire interface
    bus.write_byte_data(0x69, 0x23, 0x10)
    dps_digit = 0.01750
    
    # initialise message
    msg = Imu()
    msg.header.frame_id = "base_link"
    msg.orientation.x = 0
    msg.orientation.y = 0
    msg.orientation.z = 0
    msg.orientation.w = 1
    msg.orientation_covariance[0] = 1
    msg.orientation_covariance[4] = 1
    msg.orientation_covariance[8] = 1
    msg.linear_acceleration.x = 0
    msg.linear_acceleration.y = 0
    msg.linear_acceleration.z = 0
    msg.linear_acceleration_covariance[0] = -1
    msg.angular_velocity_covariance[0] = 1
    msg.angular_velocity_covariance[4] = 1 
    msg.angular_velocity_covariance[8] = 1 
    
    # sleep a bit to ensure gyro is ready
    time.sleep(0.5)
    
    # loop for publishing
    while not rospy.is_shutdown():
        # capture timestamp
        msg.header.stamp = rospy.get_rostime()
        # L3G4200D address, 0x69(104)
        # Read data back from 0x28(40), 2 bytes, X-Axis LSB first
        data0 = bus.read_byte_data(0x69, 0x28)
        data1 = bus.read_byte_data(0x69, 0x29)

        # Convert the data
        xGyro = data1 * 256 + data0
        if xGyro > 32767 :
                xGyro -= 65536
        xGyro = np.radians(xGyro*dps_digit)

        # L3G4200D address, 0x69(104)
        # Read data back from 0x2A(42), 2 bytes, Y-Axis LSB first
        data0 = bus.read_byte_data(0x69, 0x2A)
        data1 = bus.read_byte_data(0x69, 0x2B)

        # Convert the data
        yGyro = data1 * 256 + data0
        if yGyro > 32767 :
                yGyro -= 65536
        yGyro = np.radians(yGyro*dps_digit)

        # L3G4200D address, 0x69(104)
        # Read data back from 0x2C(44), 2 bytes, Z-Axis LSB first
        data0 = bus.read_byte_data(0x69, 0x2C)
        data1 = bus.read_byte_data(0x69, 0x2D)

        # Convert the data
        zGyro = data1 * 256 + data0
        if zGyro > 32767 :
                zGyro -= 65536
        zGyro = np.radians(zGyro*dps_digit)
       
        # update orientation by computing quaternion derivative, then integrating
        # the derivative
        # https://x-io.co.uk/downloads/madgwick_internal_report.pdf
        w = np.array([xGyro, yGyro, zGyro, 0])
        q = np.array([msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w])
        q_dot = 0.5 * quaternion_multiply(q, w)
        q_new = q + q_dot*dt
    
        # insert data to message
        msg.orientation = Quaternion(*q_new) 
        msg.angular_velocity.x = xGyro
        msg.angular_velocity.y = yGyro
        msg.angular_velocity.z = zGyro
        # publish the data
        pub.publish(msg)
        # keep loop time at constant rate
        rate.sleep()        

if __name__ == '__main__':
    try:
        gyro()
    except rospy.ROSInterruptException:
        pass
