#!/usr/bin/env python
# Software License Agreement (BSD License)
#
# Copyright (c) 2008, Willow Garage, Inc.
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#  * Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above
#    copyright notice, this list of conditions and the following
#    disclaimer in the documentation and/or other materials provided
#    with the distribution.
#  * Neither the name of Willow Garage, Inc. nor the names of its
#    contributors may be used to endorse or promote products derived
#    from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
#
# Revision $Id$

## Simple talker demo that published std_msgs/Strings messages
## to the 'chatter' topic

import rospy
import time
import random
from std_msgs.msg import String
from assignment1.msg import Coordinates

## Acquire map size parameters from launch file
map_x_max = rospy.get_param("map/x_max")
map_y_max = rospy.get_param("map/y_max")

## Acquire person's position parameters from launch file
person_x = rospy.get_param("person/x")
person_y = rospy.get_param("person/y")

def perception():
    rospy.init_node('perception_node', anonymous=True)
    voice_pub = rospy.Publisher('play_topic', String, queue_size=10)
    gesture_pub = rospy.Publisher('gesture_topic', Coordinates, queue_size=10)
    rate = rospy.Rate(200)
    #play_str = "User: <<Play>>"
    while not rospy.is_shutdown():
        #time.sleep(random.randint(2, 10))
        time.sleep(random.randint(7, 8))
        #new_pos = [random.randint(0, 10), random.randint(0, 10)]
        #command_str = "Robot, go to: %i" %new_pos
        #rospy.loginfo(command_str)
        #pub.publish(new_pos)
        #rospy.loginfo(play_str)
        choice = random.randint(1, 2)
        if choice == 1:
            voice_pub.publish("play")
        else:
            pos = Coordinates()
            pos.x = random.randint(0, map_x_max)
            pos.y = random.randint(0, map_y_max)
            gesture_pub.publish(pos)
        rate.sleep()

if __name__ == '__main__':
    try:
        perception()
    except rospy.ROSInterruptException:
        pass