#!/usr/bin/env python

## @package dog_control
# Implements a control for the robotic dog that guides it to the
# goal location. This works only when the dog's finite state machine is
# in the Normal or Sleep states

import sys
import numpy as np
import rospy
import roslib
import random
import time
import math
import assignment3.msg
from assignment3.msg import Coordinates
from geometry_msgs.msg import Twist, Point
from nav_msgs.msg import Odometry
from tf import transformations

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

## Callback function that triggers every time something is sent on the
# odom topic. It simply sets globabl variables values, and converts
# quaternions into euler angles, in order to obtain the yaw value
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

## Function used to change state of this node's control pattern
def change_state(state):
    global state_
    state_ = state

## Function used to normalize, with respect to pi, incoming angles
def normalize_angle(angle):
    if(math.fabs(angle) > math.pi):
        angle = angle - (2 * math.pi * angle) / (math.fabs(angle))
    return angle

## Step of the control algorithm which is devoted to the robotic dog's yaw
# (orientation) adjustments
def fix_yaw(des_pos):
    global yaw_, pub, yaw_precision_2_, state_, pub_over
    desired_yaw = math.atan2(des_pos.y - position_.y, des_pos.x - position_.x)
    err_yaw = normalize_angle(desired_yaw - yaw_)

    twist_msg = Twist()
    if math.fabs(err_yaw) > yaw_precision_2_:
        twist_msg.angular.z = kp_a*err_yaw
        if twist_msg.angular.z > ub_a:
            twist_msg.angular.z = ub_a
        elif twist_msg.angular.z < lb_a:
            twist_msg.angular.z = lb_a
    if rospy.get_param('ball_detected') == 0:
        pub.publish(twist_msg)

        # state change conditions
        if math.fabs(err_yaw) <= yaw_precision_2_:
            change_state(1)
    else:
        change_state(2)

## Step of the control algorithm which is devoted to the robotic dog's linear
# translation. This is carried out after the orientation of the dog has
# been properly set, but it then also checks if it has to be fixed again
def go_straight_ahead(des_pos):
    global yaw_, pub, yaw_precision_, state_, final_position, pub_over
    desired_yaw = math.atan2(des_pos.y - position_.y, des_pos.x - position_.x)
    err_yaw = desired_yaw - yaw_
    err_pos = math.sqrt(pow(des_pos.y - position_.y, 2) +
                        pow(des_pos.x - position_.x, 2))
    err_yaw = normalize_angle(desired_yaw - yaw_)
    if rospy.get_param('ball_detected') == 0:
        if err_pos > dist_precision_:
            twist_msg = Twist()
            twist_msg.linear.x = 0.3
            if twist_msg.linear.x > ub_d:
                twist_msg.linear.x = ub_d

            twist_msg.angular.z = kp_a*err_yaw
            pub.publish(twist_msg)
        else:
            final_position.x = round(position_.x)
            final_position.y = round(position_.y)
            change_state(2)

        # state change conditions
        if math.fabs(err_yaw) > yaw_precision_:
            change_state(0)

    else:
        change_state(2)

## Function used at the end of the algorithms. It stops the robot where it is
def done():
    twist_msg = Twist()
    twist_msg.linear.x = 0
    twist_msg.angular.z = 0
    pub.publish(twist_msg)

## control_topic callback. It implements the robotic dog movements,
# guiding it to the goal, then publishes the reached position on the 
# motion_over_topic
def control_cb(data):
    global pub, state_, desired_position, final_position, pub_over

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
        if state_ == 0:
            fix_yaw(desired_position_)
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


## Initializes the dog_control_node and subscribes to the control_topic. manager_listener
# keeps waiting for incoming motion requests from dog_fsm.py
def manager_listener():

    rospy.init_node('dog_control_node', anonymous=True)
    rospy.Subscriber('control_topic', Coordinates, control_cb)

    rospy.spin()


if __name__ == '__main__':
    try:
        manager_listener()
    except rospy.ROSInterruptException:
        pass