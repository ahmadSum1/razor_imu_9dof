#!/usr/bin/env python

# Modified february 2022
# By Sakib Ahmed @ahmadsum1

# Copyright (c) 2012, Tang Tiong Yew
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
#    * Redistributions of source code must retain the above copyright
#      notice, this list of conditions and the following disclaimer.
#    * Redistributions in binary form must reproduce the above copyright
#      notice, this list of conditions and the following disclaimer in the
#      documentation and/or other materials provided with the distribution.
#    * Neither the name of the Willow Garage, Inc. nor the names of its
#      contributors may be used to endorse or promote products derived from
#       this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

import rospy
import serial
import string
import math
import sys

#from time import time
from sensor_msgs.msg import Imu
from tf.transformations import quaternion_from_euler, euler_from_quaternion
from dynamic_reconfigure.server import Server
from razor_imu_9dof.cfg import imuConfig
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue

degrees2rad = math.pi/180.0
imu_yaw_calibration = 0.0

print ("modified file")

# Callback for dynamic reconfigure requests
def reconfig_callback(config, level):
    global imu_yaw_calibration
    rospy.loginfo("""Reconfigure request for yaw_calibration: %d""" %(config['yaw_calibration']))
    #if imu_yaw_calibration != config('yaw_calibration'):
    imu_yaw_calibration = config['yaw_calibration']
    rospy.loginfo("Set imu_yaw_calibration to %d" % (imu_yaw_calibration))
    return config

def euler_from_quaternion(x, y, z, w):
        """
        Convert a quaternion into euler angles (roll, pitch, yaw)
        roll is rotation around x in radians (counterclockwise)
        pitch is rotation around y in radians (counterclockwise)
        yaw is rotation around z in radians (counterclockwise)
        """
        t0 = +2.0 * (w * x + y * z)
        t1 = +1.0 - 2.0 * (x * x + y * y)
        roll_x = math.atan2(t0, t1)
     
        t2 = +2.0 * (w * y - z * x)
        t2 = +1.0 if t2 > +1.0 else t2
        t2 = -1.0 if t2 < -1.0 else t2
        pitch_y = math.asin(t2)
     
        t3 = +2.0 * (w * z + x * y)
        t4 = +1.0 - 2.0 * (y * y + z * z)
        yaw_z = math.atan2(t3, t4)
     
        return math.degrees(roll_x), math.degrees(pitch_y), math.degrees(yaw_z) # in degrees

rospy.init_node("razor_node")

imuMsg = Imu()

# Orientation covariance estimation:
# Observed orientation noise: 0.3 degrees in x, y, 0.6 degrees in z
# Magnetometer linearity: 0.1% of full scale (+/- 2 gauss) => 4 milligauss
# Earth's magnetic field strength is ~0.5 gauss, so magnetometer nonlinearity could
# cause ~0.8% yaw error (4mgauss/0.5 gauss = 0.008) => 2.8 degrees, or 0.050 radians
# i.e. variance in yaw: 0.0025
# Accelerometer non-linearity: 0.2% of 4G => 0.008G. This could cause
# static roll/pitch error of 0.8%, owing to gravity orientation sensing
# error => 2.8 degrees, or 0.05 radians. i.e. variance in roll/pitch: 0.0025
# so set all covariances the same.
imuMsg.orientation_covariance = [
0.0025 , 0 , 0,
0, 0.0025, 0,
0, 0, 0.0025
]

# Angular velocity covariance estimation:
# Observed gyro noise: 4 counts => 0.28 degrees/sec
# nonlinearity spec: 0.2% of full scale => 8 degrees/sec = 0.14 rad/sec
# Choosing the larger (0.14) as std dev, variance = 0.14^2 ~= 0.02
imuMsg.angular_velocity_covariance = [
                                        0.02, 0 , 0,
                                        0 , 0.02, 0,
                                        0 , 0 , 0.02
                                        ]

# linear acceleration covariance estimation:
# observed acceleration noise: 5 counts => 20milli-G's ~= 0.2m/s^2
# nonliniarity spec: 0.5% of full scale => 0.2m/s^2
# Choosing 0.2 as std dev, variance = 0.2^2 = 0.04
imuMsg.linear_acceleration_covariance = [
                                            0.04 , 0 , 0,
                                            0 , 0.04, 0,
                                            0 , 0 , 0.04
                                            ]

# read basic information
port = rospy.get_param('~port', '/dev/ttyUSB0')
topic = rospy.get_param('~topic', 'imu')
frame_id = rospy.get_param('~frame_id', 'base_imu_link')

# read calibration parameters

# accelerometer
accel_x_min = rospy.get_param('~accel_x_min', -250.0)
accel_x_max = rospy.get_param('~accel_x_max', 250.0)
accel_y_min = rospy.get_param('~accel_y_min', -250.0)
accel_y_max = rospy.get_param('~accel_y_max', 250.0)
accel_z_min = rospy.get_param('~accel_z_min', -250.0)
accel_z_max = rospy.get_param('~accel_z_max', 250.0)

# magnetometer
magn_x_min = rospy.get_param('~magn_x_min', -600.0)
magn_x_max = rospy.get_param('~magn_x_max', 600.0)
magn_y_min = rospy.get_param('~magn_y_min', -600.0)
magn_y_max = rospy.get_param('~magn_y_max', 600.0)
magn_z_min = rospy.get_param('~magn_z_min', -600.0)
magn_z_max = rospy.get_param('~magn_z_max', 600.0)
calibration_magn_use_extended = rospy.get_param('~calibration_magn_use_extended', False)
magn_ellipsoid_center = rospy.get_param('~magn_ellipsoid_center', [0, 0, 0])
magn_ellipsoid_transform = rospy.get_param('~magn_ellipsoid_transform', [[0, 0, 0], [0, 0, 0], [0, 0, 0]])
imu_yaw_calibration = rospy.get_param('~imu_yaw_calibration', 0.0)

# gyroscope
gyro_average_offset_x = rospy.get_param('~gyro_average_offset_x', 0.0)
gyro_average_offset_y = rospy.get_param('~gyro_average_offset_y', 0.0)
gyro_average_offset_z = rospy.get_param('~gyro_average_offset_z', 0.0)

#rospy.loginfo("%f %f %f %f %f %f", accel_x_min, accel_x_max, accel_y_min, accel_y_max, accel_z_min, accel_z_max)
#rospy.loginfo("%f %f %f %f %f %f", magn_x_min, magn_x_max, magn_y_min, magn_y_max, magn_z_min, magn_z_max)
#rospy.loginfo("%s %s %s", str(calibration_magn_use_extended), str(magn_ellipsoid_center), str(magn_ellipsoid_transform[0][0]))
#rospy.loginfo("%f %f %f", gyro_average_offset_x, gyro_average_offset_y, gyro_average_offset_z)

pub = rospy.Publisher(topic, Imu, queue_size=1)
srv = Server(imuConfig, reconfig_callback)  # define dynamic_reconfigure callback
diag_pub = rospy.Publisher('diagnostics', DiagnosticArray, queue_size=1)
diag_pub_time = rospy.get_time();

# Check your COM port and baud rate
rospy.loginfo("Opening %s...", port)
try:
    
    rospy.loginfo("IMU found at port "+port + ".")
    ser = serial.Serial(port=port, baudrate=57600, timeout=1, rtscts=True, dsrdtr=True) # For compatibility with some virtual serial ports (e.g. created by socat) in Python 2.7
except serial.serialutil.SerialException:
    rospy.logerr("IMU not found at port "+port + ". Did you specify the correct port in the launch file?")
    rospy.loginfo("IMU not found at port "+port + ". Did you specify the correct port in the launch file?")
    #exit
    sys.exit(2)

roll=0
pitch=0
yaw=0
seq=0

"""    
    sensor reports raw accel which needs to be scaled by 8.192 ==>> as milli-g(1/1000 of earth gravity). ref: https://github.com/sparkfun/SparkFun_ICM-20948_ArduinoLibrary/blob/d5ae1eba1ecbf808fca9bff0b0b6dc4e571e947c/src/ICM_20948.cpp#L191
    Convert to m/s^2. (1000 milli-g = 9.80665 m/s^2)
    raw_accel*(1/8.192)*(1/1000)*9.80665 ==>> m/s^2 
"""
accel_factor = 0.00119710083


"""
    raw Gyro data needs to be scaled by 131 to get ==>> DPS(deg/s) ==>> RPS(rad/s). ref: https://github.com/sparkfun/SparkFun_ICM-20948_ArduinoLibrary/blob/d5ae1eba1ecbf808fca9bff0b0b6dc4e571e947c/src/ICM_20948.cpp#L220-L240 and https://github.com/adafruit/Adafruit_ICM20X/blob/6ec93fee035892f780176ffa64be3af1912be559/Adafruit_ICM20948.cpp#L139-L159
    1 rad/s = 57.29578 deg/s
    raw_gyro*(1/131)*(1/57.29578) ==>> rad/s
"""
gyro_factor = 0.00013323123



rospy.loginfo("Giving the OLA IMU board 5 seconds to boot...")
rospy.sleep(5) # Sleep for 5 seconds to wait for the board to boot

for i in range(10):
    ser.flushInput()
    discard = ser.readline()


#automatic flush - NOT WORKING
ser.flushInput()  #discard old input, still in invalid format
#flush manually, as above command is not working - it breaks the serial connection
rospy.loginfo("Flushing first 20 IMU entries...")
for x in range(0, 20):
    line = bytearray(ser.readline()).decode("utf-8")
rospy.loginfo("Publishing IMU data...")
#f = open("raw_imu_data.log", 'w')

errcount = 0
while not rospy.is_shutdown():
    if (errcount > 10):
        break
    raw_data=str(ser.readline().decode("utf-8")).split(",")
    # print("raw data:"+ str(raw_data))
    words = [float(i) for i in raw_data[0:-1]]  # Delete/trim "\r\n" and convert to float

    # print("processed data:")
    # print(words)

    if len(words) != 9:
        rospy.logwarn("Bad IMU data or bad sync")
        errcount = errcount+1
        continue
    else:
        errcount = 0
        q1 = words[0]
        q2 = words[1]
        q3 = words[2]
        q123 = (q1 ** 2) + (q2 ** 2) + (q3 ** 2)   
        if q123>1:
            q123_norm = 1           #avoid sqrt of negative
            rospy.logwarn("Bad IMU data or bad sync: quaternion normalization error. dataframe:"+str(words))
        else:
            q123_norm = q123
        w = math.sqrt(1.0 - (q123_norm))
        roll_deg, pitch_deg, yaw_deg = euler_from_quaternion(q1, q2, q3, w)

        #in AHRS firmware z axis points down, in ROS z axis points up (see REP 103)
        yaw_deg = -words[0]
        yaw_deg = yaw_deg + imu_yaw_calibration
        if yaw_deg > 180.0:
            yaw_deg = yaw_deg - 360.0
        if yaw_deg < -180.0:
            yaw_deg = yaw_deg + 360.0
        yaw = yaw_deg*degrees2rad
        #in AHRS firmware y axis points right, in ROS y axis points left (see REP 103)
        pitch = -pitch_deg*degrees2rad
        roll = roll_deg*degrees2rad

        # Publish message
        # AHRS firmware accelerations are negated
        # This means y and z are correct for ROS, but x needs reversing
        imuMsg.linear_acceleration.x = -words[3] * accel_factor
        imuMsg.linear_acceleration.y = words[4] * accel_factor
        imuMsg.linear_acceleration.z = words[5] * accel_factor

        imuMsg.angular_velocity.x = words[6] * gyro_factor
        #in AHRS firmware y axis points right, in ROS y axis points left (see REP 103)
        imuMsg.angular_velocity.y = -words[7] * gyro_factor
        #in AHRS firmware z axis points down, in ROS z axis points up (see REP 103) 
        imuMsg.angular_velocity.z = -words[8] * gyro_factor

    # q = quaternion_from_euler(roll,pitch,yaw)
    imuMsg.orientation.x = q1
    imuMsg.orientation.y = q2
    imuMsg.orientation.z = q3
    imuMsg.orientation.w = w
    imuMsg.header.stamp= rospy.Time.now()
    imuMsg.header.frame_id = frame_id
    imuMsg.header.seq = seq
    seq = seq + 1
    pub.publish(imuMsg)

    if (diag_pub_time < rospy.get_time()) :
        diag_pub_time += 1
        diag_arr = DiagnosticArray()
        diag_arr.header.stamp = rospy.get_rostime()
        diag_arr.header.frame_id = '1'
        diag_msg = DiagnosticStatus()
        diag_msg.name = 'Razor_Imu'
        diag_msg.level = DiagnosticStatus.OK
        diag_msg.message = 'Received AHRS measurement'
        diag_msg.values.append(KeyValue('roll (deg)',
                                str(roll*(180.0/math.pi))))
        diag_msg.values.append(KeyValue('pitch (deg)',
                                str(pitch*(180.0/math.pi))))
        diag_msg.values.append(KeyValue('yaw (deg)',
                                str(yaw*(180.0/math.pi))))
        diag_msg.values.append(KeyValue('sequence number', str(seq)))
        diag_arr.status.append(diag_msg)
        diag_pub.publish(diag_arr)
        
ser.close

#f.close

if (errcount > 10):
    sys.exit(10)
