#!/usr/bin/env python3
import math
import rospy
from std_msgs.msg import Int16 , Int16MultiArray

ul_list = None
yaw = 0

def ultrasonic(msg):
    global ul_list
    ul_list = msg.data

def f_ultrasonic():
    return ul_list[0] if ul_list is not None else None

def r_ultrasonic():
    return ul_list[1] if ul_list is not None else None

def l_ultrasonic():
    return ul_list[2] if ul_list is not None else None

def imu(msg):
    global yaw
    yaw = msg.data

def yaw_degree():
    return yaw

def yaw_rad():
    return math.radians(yaw)

def start_node():
    rospy.init_node("sensors_reader")
    rospy.Subscriber("/ultrasonics", Int16MultiArray, ultrasonic)
    rospy.Subscriber("/yaw", Int16, imu)
    rospy.spin()

if __name__ == "__main__":
    start_node()
