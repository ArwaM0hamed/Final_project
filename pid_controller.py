#!/usr/bin/env python3
import rospy
import math
from geometry_msgs.msg import Twist
from sensors import SensorManager
import pid_controller
from Computer_Vision_Functions.cv import initialize_cv, handle_detection, switch_to_letters, switch_to_signs, process_image, run_camera_inference


# --- PID parameters ---
Kp_yaw = 0.4
Ki_yaw = 0
Kd_yaw = 0.02
Kp_d = 0.5
Ki_d = 0.01
Kd_d = 0.1

max_speed = 1

target_yaw = 0
prev_error_yaw = 0
integral_yaw = 0
prev_error_d = 0
integral_d = 0

cmd_pub = None

yaw_zero_offset = 0  # Add this global variable

def normalize_angle(angle):
    while angle > math.pi:
        angle -= 2*math.pi
    while angle < -math.pi:
        angle += 2*math.pi
    return angle

def pid_control_yaw(error, dt):
    global integral_yaw, prev_error_yaw
    integral_yaw += error * dt
    derivative = (error - prev_error_yaw) / dt if dt > 0 else 0
    output = (Kp_yaw * error) + (Ki_yaw * integral_yaw) + (Kd_yaw * derivative)
    prev_error_yaw = error
    return output

def pid_control_d(error, dt):
    global integral_d, prev_error_d
    integral_d += error * dt
    derivative = (error - prev_error_d) / dt if dt > 0 else 0
    output = (Kp_d * error) + (Ki_d * integral_d) + (Kd_d * derivative)
    prev_error_d = error
    return output

def reset_yaw_zero(sensors):
    """Call this to set the current yaw as zero reference."""
    global yaw_zero_offset
    yaw_zero_offset = sensors.yaw_rad()

def get_zeroed_yaw(sensors):
    """Returns the yaw relative to the last zero."""
    return normalize_angle(sensors.yaw_rad() - yaw_zero_offset)

def stop():
    twist = Twist()
    for _ in range(5):
        cmd_pub.publish(twist)
        rospy.sleep(0.05)

def turn_right(sensors):
    global target_yaw, integral_yaw, prev_error_yaw
    # Reset PID variables
    integral_yaw = 0
    prev_error_yaw = 0
    
    current_yaw = get_zeroed_yaw(sensors)
    target_yaw = normalize_angle(current_yaw - math.pi/2)  # -90 degrees relative to current
    
    rate = rospy.Rate(20)
    last_time = rospy.Time.now()
    start_time = last_time
    max_turn_time = 9 # 5 seconds timeout
    
    while not rospy.is_shutdown():
        now = rospy.Time.now()
        dt = (now - last_time).to_sec()
        last_time = now
        
        current_yaw = get_zeroed_yaw(sensors)
        error = normalize_angle(target_yaw - current_yaw)
        
        # Debug output
        rospy.loginfo(f"Target: {math.degrees(target_yaw):.1f}°, Current: {math.degrees(current_yaw):.1f}°, Error: {math.degrees(error):.1f}°")
        
        # Check if we've reached target angle (within ~2 degrees)        
        if math.degrees(current_yaw) >= 88:  # Reduced tolerance
            rospy.loginfo("Target angle reached!")
            stop()
            return
            
        # Timeout check
        if (now - start_time).to_sec() > max_turn_time:
            rospy.logwarn("Turn right timeout!")
            stop()
            return
            
        correction = pid_control_yaw(error, dt)
        # Less aggressive correction limiting
        correction = max(min(correction, 1.0), -1.0)
        
        # Increase turning speed
        turn_speed_factor = 5.0  # Adjust this value to control the turning speed
        correction *= turn_speed_factor
        
        twist = Twist()
        twist.angular.z = correction
        cmd_pub.publish(twist)
        rate.sleep()

def turn_left(sensors):
    global target_yaw, integral_yaw, prev_error_yaw
    # Reset PID variables
    integral_yaw = 0
    prev_error_yaw = 0
    

    current_yaw = get_zeroed_yaw(sensors)
    target_yaw = normalize_angle(current_yaw + math.pi/2)  # 90 degrees relative to current
    
    rate = rospy.Rate(20)
    last_time = rospy.Time.now()
    start_time = last_time
    max_turn_time = 9  # 5 seconds timeout
    
    while not rospy.is_shutdown():
        now = rospy.Time.now()
        dt = (now - last_time).to_sec()
        last_time = now
        
        current_yaw = get_zeroed_yaw(sensors)
        error = normalize_angle(target_yaw - current_yaw)
        
        # Debug output
        rospy.loginfo(f"Target: {math.degrees(target_yaw):.1f}°, Current: {math.degrees(current_yaw):.1f}°, Error: {math.degrees(error):.1f}°")
        
        # Check if we've reached target angle (within ~2 degrees)
        if math.degrees(current_yaw) >= -88:  # Reduced tolerance
            rospy.loginfo("Target angle reached!")
            stop()
            return
            
        # Timeout check
        if (now - start_time).to_sec() > max_turn_time:
            rospy.logwarn("Turn left timeout!")
            stop()
            return
            
        correction = pid_control_yaw(error, dt)
        # Less aggressive correction limiting
        correction = max(min(correction, 1.0), -1.0)
        
        # Increase turning speed
        turn_speed_factor = 5.0  # Adjust this value to control the turning speed
        correction *= turn_speed_factor
        
        twist = Twist()
        twist.angular.z = correction
        cmd_pub.publish(twist)
        rate.sleep()

def forward(sensors):
    global target_yaw, integral_yaw, prev_error_yaw
    
    # Reset PID variables before forward
    integral_yaw = 0
    prev_error_yaw = 0

    target_dist = 12
    # Use zeroed yaw as reference
    target_yaw = 0  # Always go "straight" from the reset point
    rate = rospy.Rate(10)
    last_time = rospy.Time.now()
    while not rospy.is_shutdown():
        now = rospy.Time.now()
        dt = (now - last_time).to_sec()
        last_time = now
        front = sensors.f_ultrasonic()
        current_yaw = get_zeroed_yaw(sensors)
        while front is None:
            rospy.logwarn("Waiting for front ultrasonic data...")
            rate.sleep()
            continue

        
        dist_error = front - target_dist
        speed = pid_control_d(dist_error, dt)
        yaw_error = normalize_angle(target_yaw - current_yaw)
        correction = pid_control_yaw(yaw_error, dt)
        if speed > max_speed:
            speed = max_speed
        if speed < 0.0:
            speed = 0.0
        twist = Twist()
        twist.linear.x = speed
        twist.angular.z = correction
        cmd_pub.publish(twist)
        if abs(dist_error) < 4 or front <12: 
            stop()
            break
        rate.sleep()

def linear_right():
        now = rospy.Time.now()
        last_time = rospy.Time.now()

        max_r_time = 2

        while not rospy.is_shutdown():
            twist = Twist()
            twist.linear.y = -0.5  # Move right
            twist.linear.x = 0.0
            twist.angular.z = 0
            now = rospy.Time.now()

            cmd_pub.publish(twist)
            if (now - last_time).to_sec() > max_r_time:
                rospy.logwarn("linear right timeout!")
                stop()
                return

def linear_left():
    now = rospy.Time.now()
    last_time = rospy.Time.now()

    max_l_time = 3

    while not rospy.is_shutdown():
        twist = Twist()
        twist.linear.y = 0.5  # Move left
        twist.linear.x = 0.0
        twist.angular.z = 0
        cmd_pub.publish(twist)
        now = rospy.Time.now()
        if (now - last_time).to_sec() > max_l_time:
            rospy.logwarn("linear left timeout!")
            stop()
            return
        
def back(sensors):   # <-- add sensors argument
    now = rospy.Time.now()
    last_time = rospy.Time.now()
    max_b_time = 2

    while not rospy.is_shutdown():
        twist = Twist()
        twist.linear.x = -0.5  # Move backward
        twist.linear.y = 0.0
        twist.angular.z = 0
        cmd_pub.publish(twist)

        now = rospy.Time.now()
        if (now - last_time).to_sec() > max_b_time:
            rospy.logwarn("backward timeout!")
            stop()

            detections = run_camera_inference(camera_index=2)

            if detections:
                action, value = handle_detection(detections)
                rospy.loginfo(f"Detections during back: {detections}")

                if action == "collect" and value:
                    rospy.loginfo(f"Collecting letter: {value}")
                    switch_to_signs()
                    pid_controller.forward(sensors)

                elif action == "skip":
                    rospy.loginfo("Penalty letter detected, skipping...")
                    pid_controller.forward(sensors)

            twist.linear.x = 0.5
            cmd_pub.publish(twist)
            if (now - last_time).to_sec() > max_b_time * 2 + 0.5:
                stop()
            if detections:
                action, value = handle_detection(detections)
                rospy.loginfo(f"Detections during back: {detections}")

                if action == "turn" and value:
                    rospy.loginfo(f"Sign detected: {value}")
                    if value == "right":
                        pid_controller.turn_right(sensors)
                    elif value == "left":
                        pid_controller.turn_left(sensors)
            return    


def start_node():
    global cmd_pub
    cmd_pub = rospy.Publisher("/cmd_vel", Twist, queue_size=10)

if __name__ == "__main__":
    rospy.init_node("pid_controller_node")
    sensors = SensorManager()
    sensors.start_node()
    start_node()
    # Example usage:
    forward(sensors)