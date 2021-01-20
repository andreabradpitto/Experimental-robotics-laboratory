#!/usr/bin/env python

## @package human
# Emulates a human agent which randomly decides to play with the robotic dog.
# The human checks if the robot is able to play, waits for it to come nearby,
# then orders it to move to a random room of the house, and finally waits for
# it come back. The game ends when the robotic dog is tired.
# All the communications sent to the robot are carried out via action a
# message publisher (over 'play_topic'), and all the orders are handled by dog_fsm.py

import rospy
import time
import random
import math
import assignment3.msg
from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped

## Acquire the list of available rooms from launch file
room_list = rospy.get_param('room_list')

## Acquire simulation speed scaling factor from launch file
sim_scale = rospy.get_param('sim_scale')

## After a pause, simulate the human behavior by randomly picking the next
# thing to do. The human can either hide or move the ball over the playing field
## After an initial pause, the human starts to check if the robot is prone to play.
# When the robotic dog is playing, the human first waits for it to come nearby, then
# sends it to a randomly chosen room among the pool of available ones, then waits for it
# to come back; then the cycle starts over. When the robot is tired, the human
# acknowledges it and takes a break too
def human():
    rospy.init_node('human_node', anonymous = True)
    rate = rospy.Rate(200)
    ## topic used to coomunicate orders to dog_fsm.py
    voice_pub = rospy.Publisher('play_topic', String, queue_size=10)
    time.sleep(random.randint(80, 120) / sim_scale)
    while not rospy.is_shutdown():
        while (rospy.get_param('state') != 'normal' or rospy.get_param('state') != 'play'):
            rate.sleep()
        if rospy.get_param('state') == 'normal':
            voice_pub.publish('play')
            rospy.loginfo('Human: I want to play')
            while (rospy.get_param('state') != 'play'):
                rate.sleep()
        # here I am assuming the human can see when the robot turns into the play state
        # (i.e. thanks to an external LED mounted on its back)
        while (rospy.get_param('state') == 'play' or rospy.get_param('state') == 'find'):
            while (rospy.get_param('play_task_status') ==  0):
                rate.sleep()
            room_choice = random.randint(0, 5)
            rospy.loginfo('Human: GoTo %s', room_list[room_choice])
            voice_pub.publish(room_list[room_choice])
            # when 'play_task_status' equals 2, the robot has completed its job
            while (rospy.get_param('play_task_status') !=  2):
                rate.sleep()
            rate.sleep()
        rospy.loginfo('Human: The robot has stopped playing')
        time.sleep(random.randint(250, 300) / sim_scale)
        rate.sleep()

if __name__ == '__main__':
    try:
        human()
    except rospy.ROSInterruptException:
        pass