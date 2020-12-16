#!/usr/bin/env python

## @package control
# Emulates physical engines and logic that the real robot uses to reach locations. 
# The emulation is carried out by simply waiting for random amounts of time

import sys
import numpy as np
from scipy.ndimage import filters
import imutils
import cv2
import rospy
import roslib
import random
import time
import math
from sensor_msgs.msg import CompressedImage
import assignment2.msg
from assignment2.msg import Coordinates
from geometry_msgs.msg import Twist, Point
from nav_msgs.msg import Odometry
from tf import transformations
#from std_msgs.msg import Float64
from std_msgs.msg import Int64

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

cnts = 0

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
    global yaw_, pub, yaw_precision_2_, state_
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

    pub.publish(twist_msg)
    #qua metti giramento anche della testa (attento che mi sa che e un angle)
    #poi ci vuole un if sia qui che in move ahead se viene vista la palla verde 

    #head_angle = twist_msg.angular.z
    #rospy.set_param('dog/head', twist_msg.angular.z)
    #pub_head.publish(head_angle)


    # state change conditions
    if math.fabs(err_yaw) <= yaw_precision_2_:
        #print ('Yaw error: [%s]' % err_yaw)
        change_state(1)


def go_straight_ahead(des_pos):
    global yaw_, pub, yaw_precision_, state_, final_position
    desired_yaw = math.atan2(des_pos.y - position_.y, des_pos.x - position_.x)
    err_yaw = desired_yaw - yaw_
    err_pos = math.sqrt(pow(des_pos.y - position_.y, 2) +
                        pow(des_pos.x - position_.x, 2))
    err_yaw = normalize_angle(desired_yaw - yaw_)
    #rospy.loginfo(err_yaw)

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


def done():
    twist_msg = Twist()
    twist_msg.linear.x = 0
    twist_msg.angular.z = 0
    pub.publish(twist_msg)


## control_topic callback. It waits for a random amount of time, simulating
# robot movement delays, then publishes the reached position on the 
# motion_over_topic
def control_cb(data):
    global pub, state_, desired_position, final_position, cnts
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
    while not (state_ == 3 or cnts > 0 or rospy.get_param('state') == 'normal'): # magari ridondante
        if state_ == 0:
            fix_yaw(desired_position_)
        elif state_ == 1:
            go_straight_ahead(desired_position_)
        elif state_ == 2:
            done()
            state_ = 3
            final_coords.x = final_position.x
            final_coords.y = final_position.y
            pub_over.publish(final_coords)
        else:
            rospy.logerr('Unknown state!')

        rate.sleep()

# ora non arrivano piu comandi a caso da altri nodi, quindi qui sopra
# se viene rilevata la palla (cnts > 0) smette subito di muoversi,
# e non ci sono cambiamenti fino a nuovo comando, che e possibile solo
# una volta tornati in normal state


def img_cb(ros_data):
    '''Callback function of subscribed topic. 
    Here images get converted and features detected'''
    global cnts

    find_counter = 0

    image_pub = rospy.Publisher("output/image_raw/compressed",
                                     CompressedImage, queue_size=1)
    vel_pub = rospy.Publisher("cmd_vel", Twist, queue_size=1)
    ball_pub = rospy.Publisher("ball_control_topic", Int64, queue_size=1)   

    #### direct conversion to CV2 ####
    np_arr = np.fromstring(ros_data.data, np.uint8)
    image_np = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)  # OpenCV >= 3.0:

    greenLower = (50, 50, 20)
    greenUpper = (70, 255, 255)

    blurred = cv2.GaussianBlur(image_np, (11, 11), 0)
    hsv = cv2.cvtColor(blurred, cv2.COLOR_BGR2HSV)
    mask = cv2.inRange(hsv, greenLower, greenUpper)
    mask = cv2.erode(mask, None, iterations=2)
    mask = cv2.dilate(mask, None, iterations=2)
    #cv2.imshow('mask', mask)
    cnts = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL,
                            cv2.CHAIN_APPROX_SIMPLE)
    cnts = imutils.grab_contours(cnts)
    center = None
    # only proceed if at least one contour was found
    if len(cnts) > 0:
        #rospy.set_param('state', 'play')
        ball_pub.publish(1)
        #qua sopra devo mandare qualcosa su un topic poi in normal metto il set
        find_counter = 0
        # find the largest contour in the mask, then use
        # it to compute the minimum enclosing circle and
        # centroid
        c = max(cnts, key=cv2.contourArea)
        ((x, y), radius) = cv2.minEnclosingCircle(c)
        M = cv2.moments(c)
        center = (int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"]))

        # only proceed if the radius meets a minimum size
        if radius > 10:
            # draw the circle and centroid on the frame,
            # then update the list of tracked points
            cv2.circle(image_np, (int(x), int(y)), int(radius),
                       (0, 255, 255), 2)
            cv2.circle(image_np, center, 5, (0, 0, 255), -1)
            vel = Twist()
            vel.angular.z = -0.002*(center[0]-400)
            vel.linear.x = -0.01*(radius-100)
            vel_pub.publish(vel)
        elif radius < 10:
            vel = Twist()
            vel.linear.x = 0.5
            vel_pub.publish(vel)
        else:
            print('qui devo mettere che gira la testa!')
            #poi devo metterlo come if head_moved == 0 printa quello e metti
            # head_move = 1. negli altri due casi sopra metti come primo
            # instruction head_moved = 0. e devo metterlo anche a inizio i len(cnts)
            # per non avere problemi. su questa ultiam cosa bisogna vedere dove si
            # potrebbe mettere (perche forse non e il posto migliore), ma dovrebbe
            # andare cosi

    elif (rospy.get_param('state') == 'play' and find_counter < 14): # per niente sicuro di questo
        vel = Twist()
        vel.angular.z = 0.5
        vel_pub.publish(vel)
        find_counter = find_counter + 1

    elif (rospy.get_param('state') == 'play' and find_counter >= 14):
        #rospy.set_param('state', 'normal')
        ball_pub.publish(2)

    # update the points queue
    # pts.appendleft(center)
    cv2.imshow('window', image_np)
    cv2.waitKey(2)


## Initializes the control_node and subscribes to the control_topic. manager_listener
# keeps waiting for incoming motion requests from command_manager.py
def manager_listener():

    rospy.init_node('control_node', anonymous=True)
    sub1 = rospy.Subscriber('control_topic', Coordinates, control_cb)
    while(not rospy.is_shutdown):
        if(rospy.get_param('state') == 'play'):
            sub1.unregister()
            sub2 = rospy.Subscriber("camera1/image_raw/compressed",
                                                   CompressedImage, img_cb,  queue_size=1)
        if(rospy.get_param('state') == 'normal' or rospy.get_param('state') == 'sleep'):
            sub2.unregister()
            sub1 = rospy.Subscriber('control_topic', Coordinates, control_cb)

        rospy.spin()


if __name__ == '__main__':
    try:
        manager_listener()
    except rospy.ROSInterruptException:
        pass