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
import functools
from rospy.numpy_msg import numpy_msg
from rospy_tutorials.msg import Floats

rospy.init_node('zero_velocity', anonymous=True)

rate = rospy.Rate(15)

zero_velocity_drone1_flag = 0.0
zero_velocity_drone2_flag = 0.0

vel_msg_drone1 = Twist()
vel_msg_drone1.linear.x = 0.0
vel_msg_drone1.linear.y = 0.0
vel_msg_drone1.linear.z = 0.0
vel_msg_drone1.angular.x = 0.0
vel_msg_drone1.angular.y = 0.0
vel_msg_drone1.angular.z = 0.0

vel_msg_drone2 = Twist()
vel_msg_drone2.linear.x = 0.0
vel_msg_drone2.linear.y = 0.0
vel_msg_drone2.linear.z = 0.0
vel_msg_drone2.angular.x = 0.0
vel_msg_drone2.angular.y = 0.0
vel_msg_drone2.angular.z = 0.0

pub_zero_vel_drone1 = rospy.Publisher("/drone1/tello/cmd_vel", Twist, queue_size=10)
pub_zero_vel_drone2 = rospy.Publisher("/drone2/tello/cmd_vel", Twist, queue_size=10)

def get_zero_velocity_drone1(data):
    global zero_velocity_drone1_flag
    zero_velocity_drone1_flag = data.data

def get_zero_velocity_drone2(data):
    global zero_velocity_drone2_flag
    zero_velocity_drone2_flag = data.data

rospy.Subscriber('/zero_velocity_drone1', Float32, callback=get_zero_velocity_drone1)
rospy.Subscriber('/zero_velocity_drone2', Float32, callback=get_zero_velocity_drone2)

while not rospy.is_shutdown():
    if (zero_velocity_drone1_flag == 1.0):
        pub_zero_vel_drone1.publish(vel_msg_drone1)
    if (zero_velocity_drone2_flag == 1.0):
        pub_zero_vel_drone2.publish(vel_msg_drone2)
    rate.sleep()