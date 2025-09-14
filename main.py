#!/usr/bin/env python3
import rospy
import sensors

if __name__ == "__main__":

    # sensor functions:
    sensors.start_node() # Start the sensors node
    sensors.f_ultrasonic() # Returns front ultrasonic distance reading
    sensors.r_ultrasonic() # Returns right ultrasonic distance reading
    sensors.l_ultrasonic() # Returns left ultrasonic distance reading
    sensors.yaw_degree() # Returns yaw angle in degree
    sensors.yaw_rad() # Returns yaw angle in radian