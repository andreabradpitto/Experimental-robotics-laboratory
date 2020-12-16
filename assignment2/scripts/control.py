#!/usr/bin/env python

## @package control
# Emulates physical engines and logic that the real robot uses to reach locations. 
# The emulation is carried out by simply waiting for random amounts of time

import sys
import numpy as np
#from scipy.ndimage import filters
#import imutils
#import cv2
import rospy
import roslib
import random
import time
import math
#from sensor_msgs.msg import CompressedImage
import assignment2.msg
from assignment2.msg import Coordinates
from geometry_msgs.msg import Twist, Point
from nav_msgs.msg import Odometry
from tf import transformations
#from std_msgs.msg import Float64

# robot state variables
position_ = Point()
final_position = Point()
yaw_ = 0
# machine state
state_ = 0
# goal
desired_position_ = Point()
desired_position_.x = 0
desired_position_.y = 0
desired_position_.z = 0
# parameters
yaw_precision_ = math.pi / 9  # +/- 20 degree allowed
yaw_precision_2_ = math.pi / 90  # +/- 2 degree allowed
dist_precision_ = 0.1
kp_a = 3.0  # It may be necessary to change the sign of this proportional controller
kp_d = 0.2
ub_a = 0.6
lb_a = -0.5
ub_d = 0.6

# publishers
pub = None

## Acquire simulation speed scaling factor from launch file
sim_scale = rospy.get_param('sim_scale')

# devo mettere i sim_scale!!!
# devo mettere in ogni "loop" un check per vedere se la palla e detected

# callbacks
def clbk_odom(msg):
    global position_
    global yaw_

    # position
    position_ = msg.pose.pose.position

    # yaw
    quaternion = (
        msg.pose.pose.orientation.x,
        msg.pose.pose.orientation.y,
        msg.pose.pose.orientation.z,
        msg.pose.pose.orientation.w)
    euler = transformations.euler_from_quaternion(quaternion)
    yaw_ = euler[2]


def change_state(state):
    global state_
    state_ = state
    #print ('State changed to [%s]' % state_)


def normalize_angle(angle):
    if(math.fabs(angle) > math.pi):
        angle = angle - (2 * math.pi * angle) / (math.fabs(angle))
    return angle


def fix_yaw(des_pos):
    global yaw_, pub, yaw_precision_2_, state_, pub_over
    desired_yaw = math.atan2(des_pos.y - position_.y, des_pos.x - position_.x)
    err_yaw = normalize_angle(desired_yaw - yaw_)
    #rospy.loginfo(err_yaw)

    #pub_head = rospy.Publisher('joint_position_controller/command', Float64, queue_size=10)

    twist_msg = Twist()
    if math.fabs(err_yaw) > yaw_precision_2_:
        twist_msg.angular.z = kp_a*err_yaw
        if twist_msg.angular.z > ub_a:
            twist_msg.angular.z = ub_a
        elif twist_msg.angular.z < lb_a:
            twist_msg.angular.z = lb_a
    #print('control 1')
    if rospy.get_param('ball_detected') == 0:
        pub.publish(twist_msg)

        # state change conditions
        if math.fabs(err_yaw) <= yaw_precision_2_:
            #print ('Yaw error: [%s]' % err_yaw)
            change_state(1)
    else:
        #twist_msg.angular.z = 0
        #pub.publish(twist_msg)

        #final_coords = Coordinates()
        #final_coords.x = position_.x
        #final_coords.y = position_.y
        #pub_over.publish(final_coords)
        change_state(2)
    #qua metti giramento anche della testa (attento che mi sa che e un angle)
    #poi ci vuole un if sia qui che in move ahead se viene vista la palla verde 

    #head_angle = twist_msg.angular.z
    #rospy.set_param('dog/head', twist_msg.angular.z)
    #pub_head.publish(head_angle)


def go_straight_ahead(des_pos):
    global yaw_, pub, yaw_precision_, state_, final_position, pub_over
    desired_yaw = math.atan2(des_pos.y - position_.y, des_pos.x - position_.x)
    err_yaw = desired_yaw - yaw_
    err_pos = math.sqrt(pow(des_pos.y - position_.y, 2) +
                        pow(des_pos.x - position_.x, 2))
    err_yaw = normalize_angle(desired_yaw - yaw_)
    #rospy.loginfo(err_yaw)
    #print('control 2')
    if rospy.get_param('ball_detected') == 0:
        if err_pos > dist_precision_:
            twist_msg = Twist()
            twist_msg.linear.x = 0.3
            if twist_msg.linear.x > ub_d:
                twist_msg.linear.x = ub_d

            twist_msg.angular.z = kp_a*err_yaw
            pub.publish(twist_msg)
        else:
            #print ('Position error: [%s]' % err_pos)
            final_position.x = round(position_.x)
            final_position.y = round(position_.y)
            change_state(2)

        # state change conditions
        if math.fabs(err_yaw) > yaw_precision_:
            #print ('Yaw error: [%s]' % err_yaw)
            change_state(0)

    else:
        #final_coords = Coordinates()
        #final_coords.x = position_.x
        #final_coords.y = position_.y
        #pub_over.publish(final_coords)
        change_state(2)

def done():
    twist_msg = Twist()
    twist_msg.linear.x = 0
    twist_msg.angular.z = 0
    pub.publish(twist_msg)

## control_topic callback. It waits for a random amount of time, simulating
# robot movement delays, then publishes the reached position on the 
# motion_over_topic
def control_cb(data):
    global pub, state_, desired_position, final_position, pub_over
    #global active_

    desired_position_.x = data.x
    desired_position_.y = data.y
    desired_position_.z = 0

    pub = rospy.Publisher('cmd_vel', Twist, queue_size=1)
    pub_over = rospy.Publisher('motion_over_topic', Coordinates, queue_size=1)
    final_coords = Coordinates()

    sub_odom = rospy.Subscriber('odom', Odometry, clbk_odom)

    state_ = 0
    rate = rospy.Rate(20)
    while not (state_ == 3 or rospy.get_param('state') == 'play'):
        #if (state_ == 0 and rospy.get_param('ball_detected') == 0):
        if state_ == 0:
            fix_yaw(desired_position_)
        #elif (state_ == 1 and rospy.get_param('ball_detected') == 0):
        elif state_ == 1:
            go_straight_ahead(desired_position_)
        elif state_ == 2:
            done()
            final_coords.x = final_position.x
            final_coords.y = final_position.y
            pub_over.publish(final_coords)
            state_ = 3
        else:
            rospy.logerr('Unknown state!')

        rate.sleep()

# ora non arrivano piu comandi a caso da altri nodi, quindi qui sopra
# se viene rilevata la palla (cnts > 0) smette subito di muoversi,
# e non ci sono cambiamenti fino a nuovo comando, che e possibile solo
# una volta tornati in normal state



## Initializes the control_node and subscribes to the control_topic. manager_listener
# keeps waiting for incoming motion requests from command_manager.py
def manager_listener():

    rospy.init_node('control_node', anonymous=True)
    rospy.Subscriber('control_topic', Coordinates, control_cb)

    rospy.spin()


if __name__ == '__main__':
    try:
        manager_listener()
    except rospy.ROSInterruptException:
        pass