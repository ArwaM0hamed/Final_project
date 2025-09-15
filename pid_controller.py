#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import Int16MultiArray, Int16

# --- PID parameters ---
Kp = 0.5
Ki = 0.01
Kd = 0.1

# --- State variables ---
target_yaw = 0      # we want robot to keep straight
current_yaw = 0
prev_error = 0
integral = 0
last_time = None   # initialize as None first

# --- Callback: update current yaw ---
def yaw_callback(msg):
    global current_yaw
    current_yaw = msg.data   # read yaw from sensor

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

# --- Main program ---
rospy.init_node("simple_pid_controller")
cmd_pub = rospy.Publisher("/cmd_vel", Twist, queue_size=10)
rospy.Subscriber("/yaw", Int16, yaw_callback)

rate = rospy.Rate(10)  # 10 Hz loop
last_time = rospy.Time.now()   

while not rospy.is_shutdown():
    now = rospy.Time.now()
    dt = (now - last_time).to_sec()
    last_time = now

    # PID error calculation
    error = target_yaw - current_yaw
    correction = pid_control(error, dt)

    # Create Twist command
    twist = Twist()
    twist.linear.x = 0.2          # forward speed
    twist.angular.z = correction  # steering correction

    cmd_pub.publish(twist)
    rate.sleep()
