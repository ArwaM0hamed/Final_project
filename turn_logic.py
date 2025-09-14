#!/usr/bin/env python3
import rospy
from std_msgs.msg import String

#global variables
score = 0

def turn-callback(msg):
    global score
    turn_status = msg.data

    if turn_status == "oneway_correct":
        score += 5
        rospy.loginfo(f"Correct one-way turn, score: {score}")
    elif turn_status == "twoway_correct":
        score += 10
        rospy.loginfo(f"Correct two-way turn, score: {score}")
    elif turn_status == "twoway_wrong":
        rospy.loginfo(f"Wrong two-way turn, score: {score}")
    else:
        rospy.loginfo(f"Unknown turn status: {turn_status}")

def main():
    rospy.init_node('turn_logic_node')
    rospy.Subscriber('/turn_status', String, turn_callback)
    rospy.loginfo("Turn logic node started")
    rospy.spin()

if __name__ == '__main__':
    main()