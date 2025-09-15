#!/usr/bin/env python3
import rospy
import pid_controller
from sensors import SensorManager

def main():
    # Initialize ROS node
    rospy.init_node("turn_node")
    
    # Initialize sensors and PID controller
    sensors = SensorManager()
    sensors.start_node()
    pid_controller.start_node()
    
    # Wait for sensor data
    if not sensors.wait_for_data():
        rospy.logerr("Failed to get sensor data")
        return
        
    # Execute turn
    pid_controller.turn_left(sensors)
    pid_controller.stop()

if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass