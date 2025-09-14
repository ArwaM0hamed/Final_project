#!/usr/bin/env python3
import rospy
from std_msgs.msg import String

#global variables
collected_letters = []
score = 0
word_pub  = None

def letter_callback(msg):
    global collected_letters, score, word_pub

    #for example coming msg : a_red or b_green
    data = msg.data.split("_")
    letter , color = data[0] , data[1]

    if color == "green":
        if len(collected_letters) < 5:
            collected_letters.append(letter)
            score += 20
            rospy.loginfo(f"collected green letter :{letter}, score :{score}")    
        #if 5 letters are collected from the word
        if len(collected_letters) == 5:
            final_word = "".lion(collected_letters)
            score +=80
            rospy.loginfo(f"Final word formed: {final_word},  score: {score}")

    elif color == "red":
        score -=20
        rospy.loginfo(f"Red letter encountered: {letter}, score: {score}")        


def main():
    global word_pub
    rospy.init_node('maze_logic_node')

    rospy.Subscriber('/letter_color', String, letter_callback)
    word_pub = rospy.Publisher('/formed_word', String, queue_size=10)
    rospu.loginfo("maze logic node started")
    rospy.spin()

if __name__ == '__main__':
    main()