#!/usr/bin/env python3
import rospy
from sensors import SensorManager
import pid_controller
from std_msgs.msg import String
from Computer_Vision_Functions.cv import initialize_cv, handle_detection, switch_to_letters, switch_to_signs, process_image, run_camera_inference

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
                score = 0
                print(f"Penalty! Red letter detected.")
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

            print("going forward 2")
            switch_to_signs()
            pid_controller.forward(sensors)
            print("Switched to sign detection mode.")
            continue
            

if __name__ == "__main__":
    main()


