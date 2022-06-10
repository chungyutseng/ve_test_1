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

rospy.init_node('calculate_velocity', anonymous=True)

rate = rospy.Rate(15)

pub_estimated_velocity = rospy.Publisher("estimated_velocity_new", Float32, queue_size=10)
pub_estimated_velocity_bound = rospy.Publisher("estimated_velocity_bound", Float32, queue_size=10)

time_duration = 0.0
min_angle = 0.0

get_start_calculating_velocity_flag = 0.0

flag = 0.0

estimated_velocity = 0.0
estimated_velocity_bound = 0.0

def get_time_duration(data):
    global time_duration
    time_duration = data.data

def get_min_angle(data):
    global min_angle
    min_angle = data.data

def get_start_calculating_velocity(data):
    global get_start_calculating_velocity_flag
    global flag
    get_start_calculating_velocity_flag =  data.data
    if (get_start_calculating_velocity_flag == 1.0):
        flag = 1.0

def calculating_estimated_velocity():
    global estimated_velocity
    global estimated_velocity_bound
    R = 0.6 * min_angle + 0.4 * time_duration
    estimated_velocity = 0.2186 * math.exp(-0.2326 * R - 0.009719)
    estimated_velocity_bound = estimated_velocity
    if (estimated_velocity < 0.3):
        estimated_velocity = 0.3
    if (estimated_velocity > 1.0):
        estimated_velocity = 1.0

rospy.Subscriber("time_duration", Float32, callback=get_time_duration)
rospy.Subscriber("min_angle", Float32, callback=get_min_angle)
rospy.Subscriber("start_calculating_velocity", Float32, callback=get_start_calculating_velocity)

while not rospy.is_shutdown():
    if (flag == 1.0):
        calculating_estimated_velocity()
        pub_estimated_velocity.publish(estimated_velocity)
        pub_estimated_velocity_bound.publish(estimated_velocity_bound)
    rate.sleep()