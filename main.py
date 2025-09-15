#!/usr/bin/env python3
import rospy
from sensors import SensorManager
import pid_controller
from std_msgs.msg import String
from Computer_Vision_Functions.cv import initialize_cv, handle_detection, switch_to_letters, switch_to_signs, process_image, run_camera_inference

# def usage_examples():

#     # sensor functions:
#     sensors.start_node() # Start the sensors node
#     sensors.f_ultrasonic() # Returns front ultrasonic distance reading
#     sensors.r_ultrasonic() # Returns right ultrasonic distance reading
#     sensors.l_ultrasonic() # Returns left ultrasonic distance reading
#     sensors.yaw_degree() # Returns yaw angle in degree
#     sensors.yaw_rad() # Returns yaw angle in radian

#     # computer vision functions (simple usage examples):
#     pub_detections = initialize_cv() # Start the CV node and get publisher for detections
#     switch_to_letters() # Switch to letter detection mode
#     switch_to_signs()   # Switch to sign detection mode

#     # Example: handle a detection result (normally from process_image)
#     detections = ["SAFE_LETTER: A", "SIGN: right"]  # Example detection list
#     action, value = handle_detection(detections)     # Returns action and value based on detections
#     print("Action:", action, "Value:", value)

#     # PID functions
#     pid_controller.start_node()
#     pid_controller.forward() # keep moving forward until the dist between front ultrasonic and the wall is 5 cm
#     pid_controller.turn_left() # turns 90 degrees left 
#     pid_controller.turn_righ() # turns 90 degrees right
#     pid_controller.stop()

# if __name__ == "__main__":

#     # start nodes
#     sensors.start_node()
#     pub_detections = initialize_cv()
#     pid_controller.start_node()

#     # main logic

# #!/usr/bin/env python3
# import rospy
# from geometry_msgs.msg import Twist
# import sensors
# from Computer_Vision_Functions.cv import initialize_cv, handle_detection, switch_to_letters

score = 0
collected_letters = []
detection_received = False
last_detection = None

def detection_callback(msg, sensors):
    global score, collected_letters, detection_received, last_detection
    
    detections = msg.data.split("\n")  # multiple detections in one message
    action, value = handle_detection(detections)
    last_detection = (action, value)
    detection_received = True

    if action == "collect" and value:
        if len(collected_letters) < 5:
            collected_letters.append(value)
            score += 20
            rospy.loginfo(f"Collected letter: {value}, Score: {score}")
            if len(collected_letters) == 5:
                word = "".join(collected_letters)
                score += 80
                rospy.loginfo(f"Final Word: {word}, Score: {score}")

    elif action == "skip":
        score -= 20
        rospy.loginfo(f"Penalty! Red letter detected. Score: {score}")

def wait_for_detection(timeout=3.0):
    """Wait for a detection or timeout"""
    global detection_received, last_detection
    detection_received = False
    start_time = rospy.Time.now()
    rate = rospy.Rate(10)
    
    while not detection_received:
        if (rospy.Time.now() - start_time).to_sec() > timeout:
            rospy.logwarn("Detection timeout!")
            return None
        rate.sleep()
    
    return last_detection

def forward_with_detection(sensors):
    """Forward movement with detection handling"""
    pid_controller.forward(sensors)
    # After stopping, check for signs/letters
    switch_to_signs()
    rospy.sleep(0.5)  # Wait for image processing
    action, value = wait_for_detection() or (None, None)
    
    if action == "turn" and value:
        if value == "right":
            pid_controller.turn_right(sensors)
            rospy.loginfo("Turned Right")
        elif value == "left":
            pid_controller.turn_left(sensors)
            rospy.loginfo("Turned Left")
    else:
        # If no sign detected, check for letters
        switch_to_letters()
        rospy.sleep(0.5)
        action, value = wait_for_detection() or (None, None)
        # Letter handling is done in detection_callback
        if action in ["collect", "skip"]:
            rospy.loginfo(f"Letter detected: {action} {value}")
    
    # Continue forward
    pid_controller.forward(sensors)

def main():
    rospy.init_node("maze_main_node")

    sensors = SensorManager()
    sensors.start_node()

    # Publisher & Subscriber
    rospy.Subscriber("/detections", String, lambda msg: detection_callback(msg, sensors))

    # Start nodes
    pid_controller.start_node()
    initialize_cv()

    rospy.loginfo("Maze main node started.")
    
    switch_to_letters()  # Start in letter mode, or switch as needed

    while not rospy.is_shutdown():
        # Run one frame of inference and get detections
        detections = run_camera_inference(camera_index=2)  # You may want to refactor run_camera_inference to yield detections per frame

        # Example: process detections
        if detections:
            action, value = handle_detection(detections)
            if action == "collect" and value:
                if value not in collected_letters:
                    print("Preparing to collect letter...")
                    collected_letters.append(value)
                    switch_to_signs()
                    score =0
                    print(f"Collected letter: {value}")
                    pid_controller.forward(sensors)
                    print("finished forward 1")
            elif action == "skip":
                score = 0
                print(f"Penalty! Red letter detected.")
            elif action == "turn" and value:
                if value == "right":
                    print("Preparing to turn right...")
                    switch_to_letters()
                    pid_controller.turn_right(sensors)
                    print("Turned Right")
                elif value == "left":
                    print("Preparing to turn left...")
                    switch_to_letters()
                    pid_controller.turn_left(sensors)
                    print("Turned Left")
            else:
                print("No relevant detection, continue navigation...")
        else:
            print("going forward 2")
            switch_to_signs()
            pid_controller.forward(sensors)
            print("finished forward 2")
            print("No detections, continue navigation...")
            print("Switched to sign detection mode.")
            continue
            

if __name__ == "__main__":
    main()


