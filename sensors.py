#!/usr/bin/env python3
import math
import rospy
from std_msgs.msg import Int16 , Int16MultiArray

ul_list = None
yaw = 0           # Global yaw variable
current_yaw = 0   # Global current_yaw variable

def ultrasonic(msg):
    global ul_list
    ul_list = list(msg.data)

def f_ultrasonic():
    if ul_list is not None and len(ul_list) > 0:
        return ul_list[0]
    return None

def r_ultrasonic():
    if ul_list is not None and len(ul_list) > 1:
        return ul_list[1]
    return None

def l_ultrasonic():
    if ul_list is not None and len(ul_list) > 2:
        return ul_list[2]
    return None

def imu(msg):
    global yaw, current_yaw
    yaw = msg.data
    current_yaw = yaw

def yaw_degree():
    global yaw
    return yaw

def yaw_rad():
    global yaw
    return math.radians(yaw)

def start_node():
    rospy.Subscriber("/yaw", Int16, imu)
    rospy.Subscriber("/ultrasonics", Int16MultiArray, ultrasonic)

if __name__ == "__main__":
    rospy.init_node("deesha")
    start_node()

    rate = rospy.Rate(10)  # 10 Hz
    while not rospy.is_shutdown():
        front = f_ultrasonic()
        if front is not None:
            rospy.loginfo(f"Front ultrasonic: {front}")
        rate.sleep()