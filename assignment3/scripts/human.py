#!/usr/bin/env python

## @package human
# Emulates a human agent which throws or hides a ball.
# The user randomly chooses what to do with the ball;
# when they have chosen, is sends a goal action to go_to_point_ball.py

import rospy
import time
import random
import math
import assignment3.msg
from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped

room_list = rospy.get_param('room_list')

## Acquire simulation speed scaling factor from launch file
sim_scale = rospy.get_param('sim_scale')

## After a pause, simulate the human behavior by randomly picking the next
# thing to do. The human can either hide or move the ball over the playing field
def human():
    rospy.init_node('human_node', anonymous = True)
    rate = rospy.Rate(200)
    voice_pub = rospy.Publisher('play_topic', String, queue_size=10)
    while not rospy.is_shutdown():
        time.sleep(random.randint(40, 70) / sim_scale)
        if rospy.get_param('state') == 'normal':
            voice_pub.publish('play')
            rospy.loginfo('Human: I want to play')
            while (rospy.get_param('state') == 'normal'):
                rate.sleep()
            #I am assuming the human can see when the robot turns into play state
            #(i.e. thanks to an external LED mounted on the robotic dog)
            while (rospy.get_param('state') == 'play' or rospy.get_param('state') == 'find'):
                while (rospy.get_param('play_task_status') ==  0):
                #while (rospy.get_param('play_task_status') ==  0 or rospy.get_param('play_task_status') ==  3):
                #magari cosi con questo while sono piu sicuro nel passaggio da find a play
                    rate.sleep()
                room_choice = random.randint(0, 5)
                rospy.loginfo('Human: GoTo %s', room_list[room_choice])
                voice_pub.publish(room_list[room_choice])
                while (rospy.get_param('play_task_status') !=  3):
                    rate.sleep()
                time.sleep(random.randint(5, 10) / sim_scale)
            rate.sleep()
            rospy.loginfo('Human: The robot has stopped playing')
            time.sleep(random.randint(250, 300) / sim_scale)
    rate.sleep()

if __name__ == '__main__':
    try:
        human()
    except rospy.ROSInterruptException:
        pass