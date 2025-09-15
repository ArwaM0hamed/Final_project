#!/usr/bin/env python3
import rospy
import sensors
import pid_controller
from std_msgs.msg import String
from Computer_Vision_Functions.cv import initialize_cv, handle_detection, switch_to_letters, switch_to_signs

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

# --- Globals ---
score = 0
collected_letters = []
cmd_pub = None

# def move_robot(linear=0.2, angular=0.0):
#     """Publish velocity commands to robot."""
#     twist = Twist()
#     twist.linear.x = linear
#     twist.angular.z = angular
#     cmd_pub.publish(twist)

def detection_callback(msg):
    """Handle detections from CV node."""
    global score, collected_letters

    detections = msg.data.split("\n")  # multiple detections in one message
    action, value = handle_detection(detections)

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

    elif action == "turn" and value:
        if value == "right":
            pid_controller.turn_right()
            rospy.loginfo("Turned Right")

        elif value == "left":
            pid_controller.turn_left5
            rospy.loginfo("Turned Left")

    else:
        pid_controller.forward()
        rospy.loginfo("Turned Left")


def main():
    global cmd_pub
    rospy.init_node("maze_main_node")

    # Publisher & Subscriber
    rospy.Subscriber("/detections", String, detection_callback)
    # Start nodes
    pid_controller.start_node()
    print("test")
    pid_controller.start_node()
    # Start CV node
    initialize_cv()
    switch_to_signs()

    rospy.loginfo("Maze main node started.")

    pid_controller.forward()


    rospy.spin()

if __name__ == "__main__":
    main()


