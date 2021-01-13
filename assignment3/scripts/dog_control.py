#!/usr/bin/env python

## @package dog_control_ball
# Implements a control for the robotic dog that lets him follow the ball
# and move the head. This works only when the dog's finite state machine is
# in the Play state

# Python libs
import sys
import time

# numpy and scipy
import numpy as np
from scipy.ndimage import filters

import imutils

# OpenCV
import cv2

# Ros libraries
import roslib
import rospy

# Ros Messages
from sensor_msgs.msg import CompressedImage
from geometry_msgs.msg import Twist

from nav_msgs.msg import Odometry

import actionlib

import assignment3.msg

from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal

blueLower = (100, 50, 50)
blueUpper = (130, 255, 255)
redLower = (0, 50, 50)
redUpper = (5, 255, 255)
greenLower = (50, 50, 50)
greenUpper = (70, 255, 255)
yellowLower = (25, 50, 50)
yellowUpper = (35, 255, 255)
magentaLower = (125, 50, 50) # non so se va bene perche si overlappa col blue
magentaUpper = (150, 255, 255)
blackLower = (0, 0, 0)
blackUpper = (5, 50, 50)

blue_solved = 0 # 0 = not yet discovered; 1 = in progress; 2 = done
red_solved = 0
green_solved = 0
yellow_solved = 0
magenta_solved = 0
black_solved = 0

## Class used to control the robotic dog during its Play state. It uses
# OpenCV in order to acquire images of the playing field and seeks a green ball.
class image_feature:

    def __init__(self):
        '''Initialize ros publisher, ros subscriber'''
        rospy.init_node('dog_control_ball_node', anonymous = True)

        # topic over which camera images get published
        self.image_pub = rospy.Publisher("output/image_raw/compressed",
                                         CompressedImage, queue_size=1)

        self.vel_pub = rospy.Publisher("cmd_vel",
                                       Twist, queue_size=1)

        # subscribed topic
        self.subscriber = rospy.Subscriber("camera1/image_raw/compressed",
                                           CompressedImage, self.callback,  queue_size=1)

    def computeCount(self, hsv, colorLower, colorUpper):
            colorMask = cv2.inRange(hsv, colorLower, colorUpper)
            colorMask = cv2.erode(colorMask, None, iterations=2)
            colorMask = cv2.dilate(colorMask, None, iterations=2)
            colorCnts = cv2.findContours(colorMask.copy(), cv2.RETR_EXTERNAL,
                                    cv2.CHAIN_APPROX_SIMPLE)
            colorCnts = imutils.grab_contours(colorCnts)
            return colorCnts

    def callback(self, ros_data):
        '''Callback function of subscribed topic. 
        Here images get converted and features detected'''
        global blue_solved, red_solved, green_solved, \
                yellow_solved, magenta_solved, black_solved

        if (rospy.get_param('state') == 'normal'):

            mb_control_client = actionlib.SimpleActionClient('move_base', MoveBaseAction)

            #### direct conversion to CV2 ####
            np_arr = np.fromstring(ros_data.data, np.uint8)
            image_np = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)  # OpenCV >= 3.0:

            blurred = cv2.GaussianBlur(image_np, (11, 11), 0)
            hsv = cv2.cvtColor(blurred, cv2.COLOR_BGR2HSV)

            blueCnts = image_feature.computeCount(self, hsv, blueLower, blueUpper)
            redCnts = image_feature.computeCount(self, hsv, redLower, redUpper)
            greenCnts = image_feature.computeCount(self, hsv, greenLower, greenUpper)
            yellowCnts = image_feature.computeCount(self, hsv, yellowLower, yellowUpper)
            magentaCnts = image_feature.computeCount(self, hsv, magentaLower, magentaUpper)
            blackCnts = image_feature.computeCount(self, hsv, blackLower, blackUpper)

            center = None
            # only proceed if at least one contour was found

            if(len(blueCnts) > 0 and blue_solved != 2 \
                     and red_solved != 1 and green_solved != 1 and yellow_solved != 1 \
                     and magenta_solved != 1 and black_solved != 1):
                rospy.set_param('new_ball_detected', 1)
                mb_control_client.cancel_all_goals()
                blue_solved = 1
                # find the largest contour in the blue mask, then use
                # it to compute the minimum enclosing circle and centroid
                c = max(blueCnts, key=cv2.contourArea)
                ((x, y), radius) = cv2.minEnclosingCircle(c)
                M = cv2.moments(c)
                center = (int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"]))

                # only proceed if the radius meets a minimum size
                if(center != 400 or radius != 100):
                    # draw the circle and centroid on the frame,
                    # then update the list of tracked points
                    # draw a yellow circle
                    cv2.circle(image_np, (int(x), int(y)), int(radius), (0, 255, 255), 2)
                    # draw a red dot
                    cv2.circle(image_np, center, 5, (0, 0, 255), -1)
                    vel = Twist()
                    vel.angular.z = -0.002 * (center[0] - 400)
                    vel.linear.x = -0.01 * (radius - 100)
                    self.vel_pub.publish(vel)
                else:
                    # the robot reached the ball: store coordinates
                    # of the corresponding room
                    pos = rospy.wait_for_message('odom', Odometry, timeout = None)
                    #pos = Odometry()
                    rospy.set_param('blue/x', pos.pose.pose.position.x)
                    rospy.set_param('blue/y', pos.pose.pose.position.y)
                    rospy.set_param('dog/x', pos.pose.pose.position.x)
                    rospy.set_param('dog/y', pos.pose.pose.position.y)
                    blue_solved = 2
                    rospy.set_param('new_ball_detected', 0)

                image_np = cv2.rotate(image_np, cv2.ROTATE_90_CLOCKWISE)
                cv2.imshow('window', image_np)
                cv2.waitKey(2)

            elif(len(redCnts) > 0 and red_solved != 2 \
                     and blue_solved != 1 and green_solved != 1 and yellow_solved != 1 \
                     and magenta_solved != 1 and black_solved != 1):
                rospy.set_param('new_ball_detected', 1)
                mb_control_client.cancel_all_goals()
                red_solved = 1
                # find the largest contour in the red mask, then use
                # it to compute the minimum enclosing circle centroid
                c = max(redCnts, key=cv2.contourArea)
                ((x, y), radius) = cv2.minEnclosingCircle(c)
                M = cv2.moments(c)
                center = (int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"]))

                # only proceed if the radius meets a minimum size
                if(center != 400 or radius != 100):
                    # draw the circle and centroid on the frame,
                    # then update the list of tracked points
                    # draw a yellow circle
                    cv2.circle(image_np, (int(x), int(y)), int(radius), (0, 255, 255), 2)
                    # draw a green dot
                    cv2.circle(image_np, center, 5, (0, 255, 0), -1)
                    vel = Twist()
                    vel.angular.z = -0.002 * (center[0] - 400)
                    vel.linear.x = -0.01 * (radius - 100)
                    self.vel_pub.publish(vel)
                else:
                    # the robot reached the ball: store coordinates
                    # of the corresponding room
                    pos = rospy.wait_for_message('odom', Odometry, timeout = None)
                    #pos = Odometry()
                    rospy.set_param('red/x', pos.pose.pose.position.x)
                    rospy.set_param('red/y', pos.pose.pose.position.y)
                    rospy.set_param('dog/x', pos.pose.pose.position.x)
                    rospy.set_param('dog/y', pos.pose.pose.position.y)
                    red_solved = 2
                    rospy.set_param('new_ball_detected', 0)

                image_np = cv2.rotate(image_np, cv2.ROTATE_90_CLOCKWISE)
                cv2.imshow('window', image_np)
                cv2.waitKey(2)

            elif(len(greenCnts) > 0 and green_solved != 2 \
                     and blue_solved != 1 and red_solved != 1 and yellow_solved != 1 \
                     and magenta_solved != 1 and black_solved != 1):
                rospy.set_param('new_ball_detected', 1)
                mb_control_client.cancel_all_goals()
                green_solved = 1
                # find the largest contour in the green mask, then use
                # it to compute the minimum enclosing circle and centroid
                c = max(greenCnts, key=cv2.contourArea)
                ((x, y), radius) = cv2.minEnclosingCircle(c)
                M = cv2.moments(c)
                center = (int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"]))

                # only proceed if the radius meets a minimum size
                if(center != 400 or radius != 100):
                    # draw the circle and centroid on the frame,
                    # then update the list of tracked points
                    # draw a yellow circle
                    cv2.circle(image_np, (int(x), int(y)), int(radius), (0, 255, 255), 2)
                    # draw a red dot
                    cv2.circle(image_np, center, 5, (0, 0, 255), -1)
                    vel = Twist()
                    vel.angular.z = -0.002 * (center[0] - 400)
                    vel.linear.x = -0.01 * (radius - 100)
                    self.vel_pub.publish(vel)
                else:
                    # the robot reached the ball: store coordinates
                    # of the corresponding room
                    pos = rospy.wait_for_message('odom', Odometry, timeout = None)
                    #pos = Odometry()
                    rospy.set_param('green/x', pos.pose.pose.position.x)
                    rospy.set_param('green/y', pos.pose.pose.position.y)
                    rospy.set_param('dog/x', pos.pose.pose.position.x)
                    rospy.set_param('dog/y', pos.pose.pose.position.y)
                    green_solved = 2
                    rospy.set_param('new_ball_detected', 0)

                image_np = cv2.rotate(image_np, cv2.ROTATE_90_CLOCKWISE)
                cv2.imshow('window', image_np)
                cv2.waitKey(2)

            elif(len(yellowCnts) > 0 and yellow_solved != 2 \
                     and blue_solved != 1 and red_solved != 1 and green_solved != 1
                     and magenta_solved != 1 and black_solved != 1):
                rospy.set_param('new_ball_detected', 1)
                mb_control_client.cancel_all_goals()
                yellow_solved = 1
                # find the largest contour in the yellow mask, then use
                # it to compute the minimum enclosing circle and centroid
                c = max(yellowCnts, key=cv2.contourArea)
                ((x, y), radius) = cv2.minEnclosingCircle(c)
                M = cv2.moments(c)
                center = (int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"]))

                # only proceed if the radius meets a minimum size
                if(center != 400 or radius != 100):
                    # draw the circle and centroid on the frame,
                    # then update the list of tracked points
                    # draw a magenta circle
                    cv2.circle(image_np, (int(x), int(y)), int(radius), (255, 0, 255), 2)
                    # draw a red dot
                    cv2.circle(image_np, center, 5, (0, 0, 255), -1)
                    vel = Twist()
                    vel.angular.z = -0.002 * (center[0] - 400)
                    vel.linear.x = -0.01 * (radius - 100)
                    self.vel_pub.publish(vel)
                else:
                    # the robot reached the ball: store coordinates
                    # of the corresponding room
                    pos = rospy.wait_for_message('odom', Odometry, timeout = None)
                    #pos = Odometry()
                    rospy.set_param('yellow/x', pos.pose.pose.position.x)
                    rospy.set_param('yellow/y', pos.pose.pose.position.y)
                    rospy.set_param('dog/x', pos.pose.pose.position.x)
                    rospy.set_param('dog/y', pos.pose.pose.position.y)
                    yellow_solved = 2
                    rospy.set_param('new_ball_detected', 0)

                image_np = cv2.rotate(image_np, cv2.ROTATE_90_CLOCKWISE)
                cv2.imshow('window', image_np)
                cv2.waitKey(2)

            elif(len(magentaCnts) > 0 and magenta_solved != 2 \
                     and blue_solved != 1 and red_solved != 1 and green_solved != 1
                     and yellow_solved != 1 and black_solved != 1):
                rospy.set_param('new_ball_detected', 1)
                mb_control_client.cancel_all_goals()
                magenta_solved = 1
                # find the largest contour in the magenta mask, then use
                # it to compute the minimum enclosing circle and centroid
                c = max(magentaCnts, key=cv2.contourArea)
                ((x, y), radius) = cv2.minEnclosingCircle(c)
                M = cv2.moments(c)
                center = (int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"]))

                # only proceed if the radius meets a minimum size
                if(center != 400 or radius != 100):
                    # draw the circle and centroid on the frame,
                    # then update the list of tracked points
                    # draw a yellow circle
                    cv2.circle(image_np, (int(x), int(y)), int(radius), (0, 255, 255), 2)
                    # draw a green dot
                    cv2.circle(image_np, center, 5, (0, 255, 0), -1)
                    vel = Twist()
                    vel.angular.z = -0.002 * (center[0] - 400)
                    vel.linear.x = -0.01 * (radius - 100)
                    self.vel_pub.publish(vel)
                else:
                    # the robot reached the ball: store coordinates
                    # of the corresponding room
                    pos = rospy.wait_for_message('odom', Odometry, timeout = None)
                    #pos = Odometry()
                    rospy.set_param('magenta/x', pos.pose.pose.position.x)
                    rospy.set_param('magenta/y', pos.pose.pose.position.y)
                    rospy.set_param('dog/x', pos.pose.pose.position.x)
                    rospy.set_param('dog/y', pos.pose.pose.position.y)
                    magenta_solved = 2
                    rospy.set_param('new_ball_detected', 0)

                image_np = cv2.rotate(image_np, cv2.ROTATE_90_CLOCKWISE)
                cv2.imshow('window', image_np)
                cv2.waitKey(2)

            elif(len(blackCnts) > 0 and black_solved != 2 \
                     and blue_solved != 1 and red_solved != 1 and green_solved != 1
                     and yellow_solved != 1 and magenta_solved != 1):
                rospy.set_param('new_ball_detected', 1)
                mb_control_client.cancel_all_goals()
                black_solved = 1
                # find the largest contour in the black mask, then use
                # it to compute the minimum enclosing circle and centroid
                c = max(blackCnts, key=cv2.contourArea)
                ((x, y), radius) = cv2.minEnclosingCircle(c)
                M = cv2.moments(c)
                center = (int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"]))

                # only proceed if the radius meets a minimum size
                if(center != 400 or radius != 100):
                    # draw the circle and centroid on the frame,
                    # then update the list of tracked points
                    # draw a yellow circle
                    cv2.circle(image_np, (int(x), int(y)), int(radius), (0, 255, 255), 2)
                    # draw a red dot
                    cv2.circle(image_np, center, 5, (0, 0, 255), -1)
                    vel = Twist()
                    vel.angular.z = -0.002 * (center[0] - 400)
                    vel.linear.x = -0.01 * (radius - 100)
                    self.vel_pub.publish(vel)
                else:
                    # the robot reached the ball: store coordinates
                    # of the corresponding room
                    pos = rospy.wait_for_message('odom', Odometry, timeout = None)
                    #pos = Odometry()
                    rospy.set_param('black/x', pos.pose.pose.position.x)
                    rospy.set_param('black/y', pos.pose.pose.position.y)
                    rospy.set_param('dog/x', pos.pose.pose.position.x)
                    rospy.set_param('dog/y', pos.pose.pose.position.y)
                    black_solved = 2
                    rospy.set_param('new_ball_detected', 0)

                image_np = cv2.rotate(image_np, cv2.ROTATE_90_CLOCKWISE)
                cv2.imshow('window', image_np)
                cv2.waitKey(2)



        elif(rospy.get_param('state') == 'find'):
            #### direct conversion to CV2 ####
            np_arr = np.fromstring(ros_data.data, np.uint8)
            image_np = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)  # OpenCV >= 3.0:

            blurred = cv2.GaussianBlur(image_np, (11, 11), 0)
            hsv = cv2.cvtColor(blurred, cv2.COLOR_BGR2HSV)

            blueCnts = image_feature.computeCount(self, hsv, blueLower, blueUpper)
            redCnts = image_feature.computeCount(self, hsv, redLower, redUpper)
            greenCnts = image_feature.computeCount(self, hsv, greenLower, greenUpper)
            yellowCnts = image_feature.computeCount(self, hsv, yellowLower, yellowUpper)
            magentaCnts = image_feature.computeCount(self, hsv, magentaLower, magentaUpper)
            blackCnts = image_feature.computeCount(self, hsv, blackLower, blackUpper)

            center = None
            # only proceed if at least one contour was found

            if(len(blueCnts) > 0 and blue_solved != 2 \
                     and red_solved != 1 and green_solved != 1 and yellow_solved != 1
                     and magenta_solved != 1 and black_solved != 1):
                rospy.set_param('new_ball_detected', 1)
                blue_solved = 1
                explore_client = actionlib.SimpleActionClient('explore', \
                     assignment3.msg.IntAction)
                #explore_client.send_goal(0)
                explore_client.cancel_all_goals()
                # find the largest contour in the blue mask, then use
                # it to compute the minimum enclosing circle and centroid
                c = max(blueCnts, key=cv2.contourArea)
                ((x, y), radius) = cv2.minEnclosingCircle(c)
                M = cv2.moments(c)
                center = (int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"]))

                # only proceed if the radius meets a minimum size
                if(center != 400 or radius != 100):
                    # draw the circle and centroid on the frame,
                    # then update the list of tracked points
                    # draw a yellow circle
                    cv2.circle(image_np, (int(x), int(y)), int(radius), (0, 255, 255), 2)
                    # draw a red dot
                    cv2.circle(image_np, center, 5, (0, 0, 255), -1)
                    vel = Twist()
                    vel.angular.z = -0.002 * (center[0] - 400)
                    vel.linear.x = -0.01 * (radius - 100)
                    self.vel_pub.publish(vel)
                else:
                    # the robot reached the ball: store coordinates
                    # of the corresponding room
                    pos = rospy.wait_for_message('odom', Odometry, timeout = None)
                    #pos = Odometry()
                    rospy.set_param('blue/x', pos.pose.pose.position.x)
                    rospy.set_param('blue/y', pos.pose.pose.position.y)
                    rospy.set_param('dog/x', pos.pose.pose.position.x)
                    rospy.set_param('dog/y', pos.pose.pose.position.y)
                    blue_solved = 2
                    rospy.set_param('new_ball_detected', 0)
                    if rospy.get_param('unknown_ball') == 0:
                        # this was the ball the robot had to find
                        rospy.set_param('unknown_ball', 100)
                    else:
                        explore_client.send_goal(1)

                image_np = cv2.rotate(image_np, cv2.ROTATE_90_CLOCKWISE)
                cv2.imshow('window', image_np)
                cv2.waitKey(2)

            elif(len(redCnts) > 0 and red_solved != 2 \
                     and blue_solved != 1 and green_solved != 1 and yellow_solved != 1
                     and magenta_solved != 1 and black_solved != 1):
                rospy.set_param('new_ball_detected', 1)
                red_solved = 1
                explore_client = actionlib.SimpleActionClient('explore', \
                     assignment3.msg.IntAction)
                #explore_client.send_goal(0)
                explore_client.cancel_all_goals()
                # find the largest contour in the red mask, then use
                # it to compute the minimum enclosing circle and centroid
                c = max(redCnts, key=cv2.contourArea)
                ((x, y), radius) = cv2.minEnclosingCircle(c)
                M = cv2.moments(c)
                center = (int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"]))

                # only proceed if the radius meets a minimum size
                if(center != 400 or radius != 100):
                    # draw the circle and centroid on the frame,
                    # then update the list of tracked points
                    # draw a yellow circle
                    cv2.circle(image_np, (int(x), int(y)), int(radius), (0, 255, 255), 2)
                    # draw a green dot
                    cv2.circle(image_np, center, 5, (0, 255, 0), -1)
                    vel = Twist()
                    vel.angular.z = -0.002 * (center[0] - 400)
                    vel.linear.x = -0.01 * (radius - 100)
                    self.vel_pub.publish(vel)
                else:
                    # the robot reached the ball: store coordinates
                    # of the corresponding room
                    pos = rospy.wait_for_message('odom', Odometry, timeout = None)
                    #pos = Odometry()
                    rospy.set_param('red/x', pos.pose.pose.position.x)
                    rospy.set_param('red/y', pos.pose.pose.position.y)
                    rospy.set_param('dog/x', pos.pose.pose.position.x)
                    rospy.set_param('dog/y', pos.pose.pose.position.y)
                    red_solved = 2
                    rospy.set_param('new_ball_detected', 0)
                    if rospy.get_param('unknown_ball') == 1:
                        # this was the ball the robot had to find
                        rospy.set_param('unknown_ball', 100)
                    else:
                        explore_client.send_goal(1)
                       
                image_np = cv2.rotate(image_np, cv2.ROTATE_90_CLOCKWISE)
                cv2.imshow('window', image_np)
                cv2.waitKey(2)

            elif(len(greenCnts) > 0 and green_solved != 2 \
                     and blue_solved != 1 and red_solved != 1 and yellow_solved != 1 \
                     and magenta_solved != 1 and black_solved != 1):
                rospy.set_param('new_ball_detected', 1)
                green_solved = 1
                explore_client = actionlib.SimpleActionClient('explore', \
                     assignment3.msg.IntAction)
                #explore_client.send_goal(0)
                explore_client.cancel_all_goals()
                # find the largest contour in the green mask, then use
                # it to compute the minimum enclosing circle and centroid
                c = max(greenCnts, key=cv2.contourArea)
                ((x, y), radius) = cv2.minEnclosingCircle(c)
                M = cv2.moments(c)
                center = (int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"]))

                # only proceed if the radius meets a minimum size
                if(center != 400 or radius != 100):
                    # draw the circle and centroid on the frame,
                    # then update the list of tracked points
                    # draw a yellow circle
                    cv2.circle(image_np, (int(x), int(y)), int(radius), (0, 255, 255), 2)
                    # draw a red dot
                    cv2.circle(image_np, center, 5, (0, 0, 255), -1)
                    vel = Twist()
                    vel.angular.z = -0.002 * (center[0] - 400)
                    vel.linear.x = -0.01 * (radius - 100)
                    self.vel_pub.publish(vel)
                else:
                    # the robot reached the ball: store coordinates
                    # of the corresponding room
                    pos = rospy.wait_for_message('odom', Odometry, timeout = None)
                    #pos = Odometry()
                    rospy.set_param('green/x', pos.pose.pose.position.x)
                    rospy.set_param('green/y', pos.pose.pose.position.y)
                    rospy.set_param('dog/x', pos.pose.pose.position.x)
                    rospy.set_param('dog/y', pos.pose.pose.position.y)
                    green_solved = 2
                    rospy.set_param('new_ball_detected', 0)
                    if rospy.get_param('unknown_ball') == 2:
                        # this was the ball the robot had to find
                        rospy.set_param('unknown_ball', 100)
                    else:
                        explore_client.send_goal(1)
 
                image_np = cv2.rotate(image_np, cv2.ROTATE_90_CLOCKWISE)
                cv2.imshow('window', image_np)
                cv2.waitKey(2)

            elif(len(yellowCnts) > 0 and yellow_solved != 2 \
                     and blue_solved != 1 and red_solved != 1 and green_solved != 1 \
                     and magenta_solved != 1 and black_solved != 1):
                rospy.set_param('new_ball_detected', 1)
                yellow_solved = 1
                explore_client = actionlib.SimpleActionClient('explore', \
                     assignment3.msg.IntAction)
                #explore_client.send_goal(0)
                explore_client.cancel_all_goals()

                # find the largest contour in the yellow mask, then use
                # it to compute the minimum enclosing circle and centroid
                c = max(yellowCnts, key=cv2.contourArea)
                ((x, y), radius) = cv2.minEnclosingCircle(c)
                M = cv2.moments(c)
                center = (int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"]))

                # only proceed if the radius meets a minimum size
                if(center != 400 or radius != 100):
                    # draw the circle and centroid on the frame,
                    # then update the list of tracked points
                    # draw a magenta circle
                    cv2.circle(image_np, (int(x), int(y)), int(radius), (255, 0, 255), 2)
                    # draw a red dot
                    cv2.circle(image_np, center, 5, (0, 0, 255), -1)
                    vel = Twist()
                    vel.angular.z = -0.002 * (center[0] - 400)
                    vel.linear.x = -0.01 * (radius - 100)
                    self.vel_pub.publish(vel)
                else:
                    # the robot reached the ball: store coordinates
                    # of the corresponding room
                    pos = rospy.wait_for_message('odom', Odometry, timeout = None)
                    #pos = Odometry()
                    rospy.set_param('yellow/x', pos.pose.pose.position.x)
                    rospy.set_param('yellow/y', pos.pose.pose.position.y)
                    rospy.set_param('dog/x', pos.pose.pose.position.x)
                    rospy.set_param('dog/y', pos.pose.pose.position.y)
                    yellow_solved = 2
                    rospy.set_param('new_ball_detected', 0)
                    if rospy.get_param('unknown_ball') == 3:
                        # this was the ball the robot had to find
                        rospy.set_param('unknown_ball', 100)
                    else:
                        explore_client.send_goal(1)
 
                image_np = cv2.rotate(image_np, cv2.ROTATE_90_CLOCKWISE)
                cv2.imshow('window', image_np)
                cv2.waitKey(2)

            elif(len(magentaCnts) > 0 and magenta_solved != 2 \
                     and blue_solved != 1 and red_solved != 1 and green_solved != 1 \
                     and yellow_solved != 1 and black_solved != 1):
                rospy.set_param('new_ball_detected', 1)
                magenta_solved = 1
                explore_client = actionlib.SimpleActionClient('explore', \
                     assignment3.msg.IntAction)
                #explore_client.send_goal(0)
                explore_client.cancel_all_goals()
                # find the largest contour in the magenta mask, then use
                # it to compute the minimum enclosing circle and centroid
                c = max(magentaCnts, key=cv2.contourArea)
                ((x, y), radius) = cv2.minEnclosingCircle(c)
                M = cv2.moments(c)
                center = (int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"]))

                # only proceed if the radius meets a minimum size
                if(center != 400 or radius != 100):
                    # draw the circle and centroid on the frame,
                    # then update the list of tracked points
                    # draw a yellow circle
                    cv2.circle(image_np, (int(x), int(y)), int(radius), (0, 255, 255), 2)
                    # draw a green dot
                    cv2.circle(image_np, center, 5, (0, 255, 0), -1)
                    vel = Twist()
                    vel.angular.z = -0.002 * (center[0] - 400)
                    vel.linear.x = -0.01 * (radius - 100)
                    self.vel_pub.publish(vel)
                else:
                    # the robot reached the ball: store coordinates
                    # of the corresponding room
                    pos = rospy.wait_for_message('odom', Odometry, timeout = None)
                    #pos = Odometry()
                    rospy.set_param('magenta/x', pos.pose.pose.position.x)
                    rospy.set_param('magenta/y', pos.pose.pose.position.y)
                    rospy.set_param('dog/x', pos.pose.pose.position.x)
                    rospy.set_param('dog/y', pos.pose.pose.position.y)
                    magenta_solved = 2
                    rospy.set_param('new_ball_detected', 0)
                    if rospy.get_param('unknown_ball') == 4:
                        # this was the ball the robot had to find
                        rospy.set_param('unknown_ball', 100)
                    else:
                        explore_client.send_goal(1)
 
                image_np = cv2.rotate(image_np, cv2.ROTATE_90_CLOCKWISE)
                cv2.imshow('window', image_np)
                cv2.waitKey(2)

            elif(len(blackCnts) > 0 and black_solved != 2 \
                     and blue_solved != 1 and red_solved != 1 and green_solved != 1 \
                     and yellow_solved != 1 and magenta_solved != 1):
                rospy.set_param('new_ball_detected', 1)
                black_solved = 1
                explore_client = actionlib.SimpleActionClient('explore', \
                     assignment3.msg.IntAction)
                #explore_client.send_goal(0)
                explore_client.cancel_all_goals()
                # find the largest contour in the black mask, then use
                # it to compute the minimum enclosing circle and centroid
                c = max(blackCnts, key=cv2.contourArea)
                ((x, y), radius) = cv2.minEnclosingCircle(c)
                M = cv2.moments(c)
                center = (int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"]))

                # only proceed if the radius meets a minimum size
                if(center != 400 or radius != 100):
                    # draw the circle and centroid on the frame,
                    # then update the list of tracked points
                    # draw a yellow circle
                    cv2.circle(image_np, (int(x), int(y)), int(radius), (0, 255, 255), 2)
                    # draw a red dot
                    cv2.circle(image_np, center, 5, (0, 0, 255), -1)
                    vel = Twist()
                    vel.angular.z = -0.002 * (center[0] - 400)
                    vel.linear.x = -0.01 * (radius - 100)
                    self.vel_pub.publish(vel)
                else:
                    # the robot reached the ball: store coordinates
                    # of the corresponding room
                    pos = rospy.wait_for_message('odom', Odometry, timeout = None)
                    #pos = Odometry()
                    rospy.set_param('black/x', pos.pose.pose.position.x)
                    rospy.set_param('black/y', pos.pose.pose.position.y)
                    rospy.set_param('dog/x', pos.pose.pose.position.x)
                    rospy.set_param('dog/y', pos.pose.pose.position.y)
                    black_solved = 2
                    rospy.set_param('new_ball_detected', 0)
                    if rospy.get_param('unknown_ball') == 5:
                        # this was the ball the robot had to find
                        rospy.set_param('unknown_ball', 100)
                    else:
                        explore_client.send_goal(1)
 
                image_np = cv2.rotate(image_np, cv2.ROTATE_90_CLOCKWISE)
                cv2.imshow('window', image_np)
                cv2.waitKey(2)



        elif(rospy.get_param('state') == 'play' or rospy.get_param('state') == 'sleep'):
            # basta un else al posto di elif qui
            #con questo assumo che il robot dia priorita ai comandi dell'uomo
            #in play se ne frega se vede nuove palle
            np_arr = np.fromstring(ros_data.data, np.uint8)
            image_np = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)  # OpenCV >= 3.0:

            image_np = cv2.rotate(image_np, cv2.ROTATE_90_CLOCKWISE)
            cv2.imshow('window', image_np)
            cv2.waitKey(2)

## Initializes the image_feature class and spins until interrupted by a keyboard command
def main(args):
    '''Initializes and cleanups ros node'''
    #ic = image_feature()
    image_feature()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print ("Shutting down ROS Image feature detector module")
    cv2.destroyAllWindows()


if __name__ == '__main__':
    main(sys.argv)