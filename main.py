#!/usr/bin/env python3
import rospy
from sensors import SensorManager
import pid_controller
from std_msgs.msg import String
import time
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
    back_flag = 0

    # Publisher & Subscriber
    rospy.Subscriber("/detections", String, lambda msg: detection_callback(msg, sensors))

    # Start nodes
    pid_controller.start_node()
    initialize_cv()

    rospy.loginfo("Maze main node started.")
    
    switch_to_letters()  # Start in letter mode, or switch as needed

    # --- Reset yaw to zero at the start ---
    pid_controller.reset_yaw_zero(sensors)

    while not rospy.is_shutdown():
        # Run one frame of inference and get detections
        detections = run_camera_inference(camera_index=2)

        if detections:
            action, value = handle_detection(detections)
            if action == "collect" and value:
                if value not in collected_letters:
                    print("Preparing to collect letter...")
                    collected_letters.append(value)
                    switch_to_signs()
                    print(f"Collected letter: {value}")
                    print("Switched to signs")
                    pid_controller.reset_yaw_zero(sensors)  # Reset before moving forward
                    pid_controller.forward(sensors)
            elif action == "skip":
                print(f"Penalty! Red letter detected.")
                print("I DIDNT TAKE THE RED LETTER AND KEPT MOVING")
                pid_controller.reset_yaw_zero(sensors)  # Reset before moving forward
                pid_controller.forward(sensors)
                
            if action == "turn" and value:
                if value == "right":
                    print("Preparing to turn right...")
                    switch_to_letters()
                    pid_controller.turn_right(sensors)  # <--- Do NOT reset yaw here
                    print("Turned Right")
                    pid_controller.reset_yaw_zero(sensors)  # <--- Reset after turn, before next forward
                elif value == "left":
                    print("Preparing to turn left...")
                    switch_to_letters()
                    pid_controller.turn_left(sensors)  # <--- Do NOT reset yaw here
                    print("Turned Left")
                    pid_controller.reset_yaw_zero(sensors)  # <--- Reset after turn, before next forward
            else:
                print("No relevant detection, continue navigation...")

        else:
            action, value = handle_detection(detections)
            switch_to_signs()
            time.sleep(0.5)
            switch_to_letters()
            print("here 0")
            time.sleep(0.5)
            while not detections:
                print("here 1")
                pid_controller.reset_yaw_zero(sensors)
                pid_controller.forward(sensors)
                switch_to_signs()
                time.sleep(0.5)
                switch_to_letters()
                time.sleep(0.5)
                if detections:
                    break
                print("here 2 ")
                right = sensors.r_ultrasonic()
                left = sensors.l_ultrasonic()
                if right < 20 and left > 20:
                    pid_controller.back(sensors)
                    pid_controller.turn_left(sensors)
                elif left < 20 and right > 20:
                    pid_controller.back(sensors)
                    pid_controller.turn_right(sensors)
                else:
                    if back_flag == 0:
                        pid_controller.back(sensors)
                        back_flag += 1
                    else: 
                        back_flag = 0
                        pid_controller.turn_left(sensors)


                print("here 3 ")
            else:
                if right < 20 and left > 20:
                    pid_controller.turn_left(sensors)
                elif left < 20 and right > 20:
                    pid_controller.turn_right(sensors)

                continue            

if __name__ == "__main__":
    
    main()


