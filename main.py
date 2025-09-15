#!/usr/bin/env python3
import rospy
import sensors
import pid_controller
from Computer_Vision_Functions.cv import initialize_cv, handle_detection, switch_to_letters, switch_to_signs

def usage_examples():

    # sensor functions:
    sensors.start_node() # Start the sensors node
    sensors.f_ultrasonic() # Returns front ultrasonic distance reading
    sensors.r_ultrasonic() # Returns right ultrasonic distance reading
    sensors.l_ultrasonic() # Returns left ultrasonic distance reading
    sensors.yaw_degree() # Returns yaw angle in degree
    sensors.yaw_rad() # Returns yaw angle in radian

    # computer vision functions (simple usage examples):
    pub_detections = initialize_cv() # Start the CV node and get publisher for detections
    switch_to_letters() # Switch to letter detection mode
    switch_to_signs()   # Switch to sign detection mode

    # Example: handle a detection result (normally from process_image)
    detections = ["SAFE_LETTER: A", "SIGN: right"]  # Example detection list
    action, value = handle_detection(detections)     # Returns action and value based on detections
    print("Action:", action, "Value:", value)

    # PID functions
    pid_controller.start_node()
    pid_controller.forward()
    pid_controller.turn_left() # turns 90 degrees left 
    pid_controller.turn_righ() # turns 90 degrees right
    pid_controller.stop()

if __name__ == "__main__":

    # start nodes
    sensors.start_node()
    pub_detections = initialize_cv()
    pid_controller.start_node()

    # main logic
    switch_to_signs()
    if "right" : # should be updated by moustafa to use yolo model _______________________________




