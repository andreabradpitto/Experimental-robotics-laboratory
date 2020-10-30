#!/usr/bin/env python

## @package control
# Emulates physical engines and logic that the real robot uses to reach locations. 
# The emulation is carried out by simply waiting for random amounts of time

import rospy
import random
import time
from std_msgs.msg import String
from assignment1.msg import Coordinates

## Acquire simulation speed scaling factor from launch file
sim_speed = rospy.get_param('sim_speed')

## control_topic callback. It waits for a random amount of time, simulating
# robot movement delays, then publishes the reached position on the 
# motion_over_topic
def control_cb(data):
    time.sleep(random.randint(2, 3) * sim_speed)
    pub = rospy.Publisher('motion_over_topic', Coordinates, queue_size=10)
    pub.publish(data)

## Initializes the control_node and subscribes to the control_topic. manager_listener
# keeps waiting for incoming motion requests from command_manager.py
def manager_listener():

    # In ROS, nodes are uniquely named. If two nodes with the same
    # name are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    rospy.init_node('control_node', anonymous=True)
    rospy.Subscriber('control_topic', Coordinates, control_cb)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    try:
        manager_listener()
    except rospy.ROSInterruptException:
        pass