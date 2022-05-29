#!/usr/bin/env python3
import rospy
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
import numpy as np
import cv2
import cv2.aruco as aruco
import math
from std_msgs.msg import Float32
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
import time
from std_msgs.msg import Empty as EmptyMsg
from sensor_msgs.msg import Imu as ImuMsg
import tf
import math
from rospy.numpy_msg import numpy_msg
from rospy_tutorials.msg import Floats

rospy.init_node('velocity_estimator', anonymous=True)

rate = rospy.Rate(15)

# roll pitch yaw
# attitude_data = np.zeros((3,), dtype=np.float32)

velocity_estimator_flag = 0.0
velocity_estimator_flag_finish = 0.0
finish_velocity_estimator_flag = 0.0

total_pitch_angle = 0.0
count = 0.0
averaged_pitch_angle = 0.0
estimated_velocity = 0.0

flag = 0

pub_estimated_velocity = rospy.Publisher("estimated_velocity", Float32, queue_size=10)

def get_imu_message(imu_msg):
    global total_pitch_angle, count
    temp_imu = imu_msg.data
    # attitude_data[0] = temp_imu[3] # roll
    # attitude_data[1] = temp_imu[4] # pitch
    # attitude_data[2] = temp_imu[5] # yaw

    if (velocity_estimator_flag == 1.0):
        total_pitch_angle = total_pitch_angle + temp_imu[4]
        count = count + 1.0

def get_velocity_estimator(data):
    global velocity_estimator_flag, velocity_estimator_flag_finish
    velocity_estimator_flag = data.data
    if (velocity_estimator_flag == 1.0):
        velocity_estimator_flag_finish = 1.0
        print("CALCULATING AVERAGING ANGLE")
    if ((velocity_estimator_flag == 0.0) and (velocity_estimator_flag_finish == 1.0)):
        velocity_estimator_flag_finish = 0.0
        print("FINISH CALCULATING AVERAGING ANGLE")

def get_finish_velocity_estimator(data):
    global finish_velocity_estimator_flag
    global averaged_pitch_angle, estimated_velocity
    finish_velocity_estimator_flag = data.data
    if (finish_velocity_estimator_flag == 1.0):
        averaged_pitch_angle = (total_pitch_angle / count)
        estimated_velocity = averaged_pitch_angle 

rospy.Subscriber("repub_imu", numpy_msg(Floats), callback=get_imu_message)
rospy.Subscriber("velocity_estimator", Float32, callback=get_velocity_estimator)
rospy.Subscriber("finish_velocity_estimator", Float32, callback=get_finish_velocity_estimator)

while not rospy.is_shutdown():
    if (finish_velocity_estimator_flag == 1.0):
        pub_estimated_velocity.publish(estimated_velocity)
    rate.sleep()