#!/usr/bin/env python3
import math
import rospy
from std_msgs.msg import Int16, Int16MultiArray

class SensorManager:
    def __init__(self):
        self.ul_list = None
        self.yaw = 0
        self.current_yaw = 0
        self.data_received = False
    
    def ultrasonic_callback(self, msg):
        self.ul_list = list(msg.data)
        self.data_received = True

    def imu_callback(self, msg):
        self.yaw = msg.data
        self.current_yaw = self.yaw

    def f_ultrasonic(self):
        if self.ul_list is not None and len(self.ul_list) > 0:
            return self.ul_list[0]
        return None

    def r_ultrasonic(self):
        if self.ul_list is not None and len(self.ul_list) > 1:
            return self.ul_list[1]
        return None

    def l_ultrasonic(self):
        if self.ul_list is not None and len(self.ul_list) > 2:
            return self.ul_list[2]
        return None

    def yaw_degree(self):
        return self.yaw

    def yaw_rad(self):
        return math.radians(self.yaw)

    def start_node(self):
        rospy.Subscriber("/yaw", Int16, self.imu_callback)
        rospy.Subscriber("/ultrasonics", Int16MultiArray, self.ultrasonic_callback)

    def wait_for_data(self, timeout=5.0):
        """Wait for sensor data to be received"""
        start_time = rospy.Time.now()
        rate = rospy.Rate(10)  # 10Hz
        while not self.data_received:
            if (rospy.Time.now() - start_time).to_sec() > timeout:
                rospy.logwarn("Timeout waiting for sensor data!")
                return False
            rate.sleep()
        return True

# Optional: for standalone testing
if __name__ == "__main__":
    rospy.init_node("deesha")
    sensors = SensorManager()
    sensors.start_node()
    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        front = sensors.f_ultrasonic()
        if front is not None:
            rospy.loginfo(f"Front ultrasonic: {front}")
        rate.sleep()