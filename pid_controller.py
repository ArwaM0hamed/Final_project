#!/usr/bin/env python3
import rospy
import sensors
import math
from geometry_msgs.msg import Twist

# --- PID parameters ---
Kp_yaw = 0.5
Ki_yaw = 0.01
Kd_yaw = 0.1
Kp_d = 0.5
Ki_d = 0.01
Kd_d = 0.1

# --- other parameters ---
max_speed = 0.3

# --- State variables ---
target_yaw = 0
prev_error_yaw = 0
integral_yaw = 0

prev_error_d = 0
integral_d = 0

# --- Globals ---
cmd_pub = None   # publisher will be initialized in start_node()

# wrap the angle between [-pi, pi]
def normalize_angle(angle):
    while angle > math.pi:
        angle -= 2*math.pi
    while angle < -math.pi:
        angle += 2*math.pi
    return angle

# --- PID function ---
def pid_control_yaw(error, dt):
    global integral_yaw, prev_error_yaw

    # integral term
    integral_yaw += error * dt

    # Derivative term
    derivative = (error - prev_error_yaw) / dt if dt > 0 else 0

    # PID formula
    output = (Kp_yaw * error) + (Ki_yaw * integral_yaw) + (Kd_yaw * derivative)

    # Save error for next loop
    prev_error_yaw = error
    return output

def pid_control_d(error,dt):
    global integral_d, prev_error_d

    # integral term
    integral_d += error * dt

    # Derivative term
    derivative = (error - prev_error_d) / dt if dt > 0 else 0

    # PID formula
    output = (Kp_d * error) + (Ki_d * integral_d) + (Kd_d * derivative)

    # Save error for next loop
    prev_error_d = error
    return output


# --- Movement functions ---
def forward():
    global target_yaw

    target_dist = 5 # stop at 5 cm
    target_yaw = sensors.yaw_rad()  # keep current heading as reference

    rate = rospy.Rate(10)
    last_time = rospy.Time.now()

    while not rospy.is_shutdown():
        # --- Time step ---
        now = rospy.Time.now()
        dt = (now - last_time).to_sec()
        last_time = now

        # Get sensor values
        front = sensors.f_ultrasonic()
        current_yaw = sensors.yaw_rad()

        # Wait for valid data
        if front is None:
            rospy.logwarn("Waiting for front ultrasonic data...")
            rate.sleep()
            continue

        # distance pid
        dist_error = front - target_dist   # positive if too far
        speed = pid_control_d(dist_error, dt)

        # yaw pid to stay straight
        yaw_error = normalize_angle(target_yaw - current_yaw)
        correction = pid_control_yaw(yaw_error, dt)

        # --- Clamp linear speed ---
        if speed > max_speed:
            speed = max_speed
        if speed < 0.0:
            speed = 0.0

        twist = Twist()
        twist.linear.x = speed
        twist.angular.z = correction
        cmd_pub.publish(twist)

        if abs(dist_error) < 1:   # 1 cm threshold
            stop()
            break
        rate.sleep()


def stop():
    twist = Twist()
    for _ in range(5): # send zeros multiple time 
        cmd_pub.publish(twist)
        rospy.sleep(0.05)


def turn_right():
    """Rotate 90° right"""
    global target_yaw
    current_yaw = sensors.yaw_rad()
    target_yaw = normalize_angle(current_yaw - math.pi/2)

    rate = rospy.Rate(10)
    last_time = rospy.Time.now()

    while not rospy.is_shutdown():
        now = rospy.Time.now()
        dt = (now - last_time).to_sec()
        last_time = now

        current_yaw = sensors.yaw_rad()
        error = normalize_angle(target_yaw - current_yaw)
        if abs(error) < 0.035:  # within 2 degrees
            stop()
            break

        correction = pid_control_yaw(error, dt)
        twist = Twist()
        twist.angular.z = correction
        cmd_pub.publish(twist)
        rate.sleep()

def turn_left():
    """Rotate 90° left"""
    global target_yaw
    current_yaw = sensors.yaw_rad()
    target_yaw = normalize_angle(current_yaw + math.pi/2)

    rate = rospy.Rate(10)
    last_time = rospy.Time.now()

    while not rospy.is_shutdown():
        now = rospy.Time.now()
        dt = (now - last_time).to_sec()
        last_time = now

        current_yaw = sensors.yaw_rad()
        error = normalize_angle(target_yaw - current_yaw)
        if abs(error) < 0.035:  # within 2 degrees
            stop()
            break

        correction = pid_control_yaw(error, dt)
        twist = Twist()
        twist.angular.z = correction
        cmd_pub.publish(twist)
        rate.sleep()

def start_node():
    global cmd_pub

    cmd_pub = rospy.Publisher("/cmd_vel", Twist, queue_size=10)


    
# --- Main program ---
if __name__ == "__main__":
    start_node()