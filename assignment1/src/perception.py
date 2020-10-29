#!/usr/bin/env python

import rospy
import time
import random
from std_msgs.msg import String
from assignment1.msg import Coordinates

## Acquire map size parameters from launch file
map_x_max = rospy.get_param('map/x_max')
map_y_max = rospy.get_param('map/y_max')

def perception():
    rospy.init_node('perception_node', anonymous=True)
    voice_pub = rospy.Publisher('play_topic', String, queue_size=10)
    gesture_pub = rospy.Publisher('gesture_topic', Coordinates, queue_size=10)
    rate = rospy.Rate(200)
    choice = Coordinates()
    while not rospy.is_shutdown():
        time.sleep(random.randint(5, 8))
        voice_pub.publish('play')
        rospy.loginfo('User: I want to play')
        time.sleep(random.randint(1, 3))
        if rospy.get_param('state') == 'normal':
            choice.x = random.randint(0, map_x_max)
            choice.y = random.randint(0, map_y_max)
            gesture_pub.publish(choice)
            time.sleep(3)
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