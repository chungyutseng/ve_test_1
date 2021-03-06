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
from rospy.numpy_msg import numpy_msg
from rospy_tutorials.msg import Floats
from tello_driver.msg import TelloStatus
import functools

rospy.init_node('reference_publisher', anonymous=True)

rate = rospy.Rate(15)

desired_pose_drone_one = np.zeros((4,), dtype=np.float32)
desired_pose_drone_two = np.zeros((4,), dtype=np.float32)

stage_one_drone_one_flag = 0.0
stage_one_drone_two_flag = 0.0
stage_two_drone_one_flag = 0.0
stage_three_drone_two_flag = 0.0

small_marker = 0.0

pub_desired_pose_drone_one = rospy.Publisher("/drone1/desired_pose", numpy_msg(Floats), queue_size=10)
pub_desired_pose_drone_two = rospy.Publisher("/drone2/desired_pose", numpy_msg(Floats), queue_size=10)

def get_stage_one_drone_one(data):
    global stage_one_drone_one_flag
    stage_one_drone_one_flag = data.data

def get_stage_one_drone_two(data):
    global stage_one_drone_two_flag
    stage_one_drone_two_flag = data.data

def get_stage_two_drone_one(data):
    global stage_two_drone_one_flag
    stage_two_drone_one_flag = data.data

def get_stage_three_drone_two(data):
    global stage_three_drone_two_flag
    stage_three_drone_two_flag = data.data

def stage_one_reference():
    desired_pose_drone_one[0] = 0.0
    desired_pose_drone_one[1] = -0.45
    desired_pose_drone_one[2] = 1.2
    desired_pose_drone_one[3] = 0
    
    if small_marker == 1.0:
        desired_pose_drone_two[0] =  0.03
        desired_pose_drone_two[1] = -0.70
        desired_pose_drone_two[2] =  0.10
        desired_pose_drone_two[3] =  0.0
    else:
        desired_pose_drone_two[0] =  0.0
        desired_pose_drone_two[1] = -1.05
        desired_pose_drone_two[2] =  1.5
        desired_pose_drone_two[3] =  0.0

def stage_two_reference():
    desired_pose_drone_one[0] = 0.0
    desired_pose_drone_one[1] = 0.4
    desired_pose_drone_one[2] = 1.2
    desired_pose_drone_one[3] = 0

def stage_three_reference():
    desired_pose_drone_two[0] =  0.0
    desired_pose_drone_two[1] = -0.2
    desired_pose_drone_two[2] =  1.5
    desired_pose_drone_two[3] =  0.0

rospy.Subscriber("/stage_one_drone_one", Float32, callback=get_stage_one_drone_one)
rospy.Subscriber("/stage_one_drone_two", Float32, callback=get_stage_one_drone_two)
rospy.Subscriber("/stage_two_drone_one", Float32, callback=get_stage_two_drone_one)
rospy.Subscriber("/stage_three_drone_two", Float32, callback=get_stage_three_drone_two)

while not rospy.is_shutdown():
    if (stage_one_drone_one_flag == 1.0):
        stage_one_reference()
        pub_desired_pose_drone_one.publish(desired_pose_drone_one)
    if (stage_one_drone_two_flag == 1.0):
        stage_one_reference()
        pub_desired_pose_drone_two.publish(desired_pose_drone_two)
    if (stage_two_drone_one_flag == 1.0):
        stage_two_reference()
        pub_desired_pose_drone_one.publish(desired_pose_drone_one)
    if (stage_three_drone_two_flag == 1.0):
        stage_three_reference()
        pub_desired_pose_drone_two.publish(desired_pose_drone_two)
    rate.sleep()



