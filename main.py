#!/usr/bin/env python3
import rospy
from sensors import SensorManager
import pid_controller
from std_msgs.msg import String
import time
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
    
    return last_detection

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

        if detections:
            action, value = handle_detection(detections)
            if action == "collect" and value:
                if value not in collected_letters:
                    print("Preparing to collect letter...")
                    collected_letters.append(value)
                    switch_to_signs()
                    print(f"Collected letter: {value}")
                    print("Switched to signs")
                    pid_controller.forward(sensors)
            elif action == "skip":
                print(f"Penalty! Red letter detected.")
                print("I DIDNT TAKE THE RED LETTER AND KEPT MOVING")
                pid_controller.forward(sensors)
            if action == "turn" and value:
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
            action, value = handle_detection(detections)
            switch_to_signs()
            time.sleep(1.5)
            switch_to_letters()
            time.sleep(1.5)
            while not detections:
                pid_controller.forward(sensors)
                switch_to_signs()
                time.sleep(1.5)
                switch_to_letters()
                time.sleep(1.5)
                pid_controller.turn_right(sensors)
                switch_to_signs()
                time.sleep(1.5)
                switch_to_letters()
                time.sleep(1.5)
                pid_controller.forward(sensors)
                switch_to_signs()
                time.sleep(1.5)
                switch_to_letters()
                time.sleep(1.5)
                pid_controller.turn_left(sensors)
                switch_to_signs()
                time.sleep(1.5)
                switch_to_letters()
                time.sleep(1.5)
                pid_controller.turn_left(sensors)
                switch_to_signs()
                time.sleep(1.5)
                switch_to_letters()
                time.sleep(1.5)
                pid_controller.forward(sensors)
            else:
                continue
            

if __name__ == "__main__":
    main()


