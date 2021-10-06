#!/usr/bin/env python

## @package human
# Emulates a human agent which randomly decides to play with the robotic dog.
# The human checks if the robot is able to play; if so, waits for it to come nearby,
# then orders it to move to a random room of the house, and finally waits for
# it come back. The game ends when the robotic dog gets tired.
# All the communications sent to the robot are carried out via a message
# publisher (over the 'play_topic'), and all the orders are handled by dog_fsm.py

import rospy
import time
import random
from std_msgs.msg import String

## Acquire the dictionary of the room and colored ball pairings from launch file
room_dict = rospy.get_param('room_dict')

## Acquire simulation speed scaling factor from launch file
sim_scale = rospy.get_param('sim_scale')

## After an initial pause, the human starts to check if the robot is prone to play.
# When the robotic dog is playing, the human first waits for it to come nearby, then
# sends it to a randomly chosen room among the pool of available ones, then waits for it
# to come back; then the cycle starts over. When the robot is tired, the human
# acknowledges it and takes a break too
def human():
    rospy.init_node('human_node', anonymous = True)
    rate = rospy.Rate(200)
    ## topic used to communicate orders to dog_fsm.py
    voice_pub = rospy.Publisher('play_topic', String, queue_size=10)
    time.sleep(random.randint(80, 120) / sim_scale)
    while not rospy.is_shutdown():
        while (rospy.get_param('state') == 'sleep'):
            rate.sleep()
        while (rospy.get_param('state') != 'normal'):
            rate.sleep()
        rospy.loginfo('Human: I want to play')
        voice_pub.publish('play')
        while (rospy.get_param('state') != 'play'):
            rate.sleep()
        # here I am assuming the human can see when the robot turns into the play state
        # (i.e. thanks to an external LED mounted on its back)
        while (rospy.get_param('state') == 'play'):
            while (rospy.get_param('play_task_ready') == 0):
                if (rospy.get_param('state') == 'sleep'):
                    break
                rate.sleep()
            room_choice = random.choice(room_dict.keys())
            rospy.loginfo('Human: Go to the %s', room_choice)
            voice_pub.publish(room_choice)
            while (rospy.get_param('play_task_done') ==  0):
                if (rospy.get_param('state') == 'sleep'):
                    break
                rate.sleep()
            rate.sleep()
        rospy.loginfo('Human: The robot has stopped playing')
        time.sleep(random.randint(250, 350) / sim_scale)
        rate.sleep()

if __name__ == '__main__':
    try:
        human()
    except rospy.ROSInterruptException:
        pass
