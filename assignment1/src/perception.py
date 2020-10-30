#!/usr/bin/env python

## @package perception
# Emulates an user's voice command and pointing gestures.
# The user randomly asks the robot to <<play>> and issues a location to reach

import rospy
import time
import random
from std_msgs.msg import String
from assignment1.msg import Coordinates

## Acquire maximum x-axis parameter from launch file
map_x_max = rospy.get_param('map/x_max')
## Acquire maximum y-axis parameter from launch file
map_y_max = rospy.get_param('map/y_max')

## Acquire simulation speed scaling factor from launch file
sim_speed = rospy.get_param('sim_speed')

## Simulate the sensors capturing user's <<play>> requests and subsequent
# pointing gestures. The user is assumed to be aware of the robot's current
# state (e.g. via LEDs on the robot itself) and asks to <<play>> only if
# MiRo is not sleeping or already playing, i.e. it is in Normal state
def perception():
    rospy.init_node('perception_node', anonymous=True)
    voice_pub = rospy.Publisher('play_topic', String, queue_size=10)
    gesture_pub = rospy.Publisher('gesture_topic', Coordinates, queue_size=10)
    rate = rospy.Rate(200)
    choice = Coordinates()
    while not rospy.is_shutdown():
        time.sleep(random.randint(5, 8) * sim_speed)
        voice_pub.publish('play')
        rospy.loginfo('User: I want to play')
        time.sleep(random.randint(1, 3) * sim_speed)
        if rospy.get_param('state') == 'normal':
            choice.x = random.randint(0, map_x_max)
            choice.y = random.randint(0, map_y_max)
            gesture_pub.publish(choice)
            rospy.loginfo('*The user points to %i %i*', choice.x, choice.y)
            time.sleep(3  * sim_speed)
            while rospy.get_param('state') == 'play':
                if rospy.get_param('MiRo/x') == rospy.get_param('person/x') and \
                    rospy.get_param('MiRo/y') == rospy.get_param('person/y'):
                    choice.x = random.randint(0, map_x_max)
                    choice.y = random.randint(0, map_y_max)
                    gesture_pub.publish(choice)
        rate.sleep()

if __name__ == '__main__':
    try:
        perception()
    except rospy.ROSInterruptException:
        pass