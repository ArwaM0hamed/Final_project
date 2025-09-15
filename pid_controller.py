#!/usr/bin/env python3
import rospy
import sensors
import math
from geometry_msgs.msg import Twist

# --- PID parameters ---
Kp = 0.5
Ki = 0.01
Kd = 0.1

# --- State variables ---
target_yaw = 0
prev_error = 0
integral = 0

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
def pid_control(error, dt):
    global integral, prev_error

    # Integral term
    integral += error * dt

    # Derivative term
    derivative = (error - prev_error) / dt if dt > 0 else 0

    # PID formula
    output = (Kp * error) + (Ki * integral) + (Kd * derivative)

    # Save error for next loop
    prev_error = error
    return output

# --- Movement functions ---
def forward():
    """Go forward with constant speed"""
    twist = Twist()
    twist.linear.x = 0.2
    twist.angular.z = 0.0
    cmd_pub.publish(twist)

def stop():
    """Stop robot"""
    twist = Twist()
    cmd_pub.publish(twist)

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

        correction = pid_control(error, dt)
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
        error = normalize_angle(target_yaw + current_yaw)
        if abs(error) < 0.035:  # within 2 degrees
            stop()
            break

        correction = pid_control(error, dt)
        twist = Twist()
        twist.angular.z = correction
        cmd_pub.publish(twist)
        rate.sleep()

def start_node():
    global cmd_pub
    rospy.init_node("simple_pid_controller")
    cmd_pub = rospy.Publisher("/cmd_vel", Twist, queue_size=10)
    rospy.spin()

    
# --- Main program ---
if __name__ == "__main__":
    start_node()