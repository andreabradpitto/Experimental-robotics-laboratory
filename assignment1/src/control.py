#!/usr/bin/env python

import rospy
import random
import time
from std_msgs.msg import String
from assignment1.msg import Coordinates

def control_cb(data):
    pub = rospy.Publisher('motion_over_topic', Coordinates, queue_size=10)
    #time.sleep(random.randint(1, 3))
    time.sleep(2)
    pub.publish(data)

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