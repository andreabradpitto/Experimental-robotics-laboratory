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

from std_msgs.msg import Int64, Float64

## variable used as a time before the robotic dog gets back to the Normal state
find_counter = 0

## variable used to acknowledge when the head has been moved in that situation
head_moved = 0

## Acquire simulation speed scaling factor from launch file
sim_scale = rospy.get_param('sim_scale')

## Class used to control the robotic dog during its Play state. It uses
# OpenCV in order to acquire images of the playing field and seeks a green ball.
class image_feature:

    def __init__(self):
        '''Initialize ros publisher, ros subscriber'''
        rospy.init_node('dog_control_ball_node', anonymous=True)

        # topic where we publish
        self.image_pub = rospy.Publisher("output/image_raw/compressed",
                                         CompressedImage, queue_size=1)
        self.vel_pub = rospy.Publisher("cmd_vel",
                                       Twist, queue_size=1)

        self.ball_pub = rospy.Publisher("ball_control_topic", Int64, queue_size=1) 

        # subscribed Topic
        self.subscriber = rospy.Subscriber("camera1/image_raw/compressed",
                                           CompressedImage, self.callback,  queue_size=1)

        self.head_pub = rospy.Publisher("joint_position_controller/command", Float64, queue_size=1)

    def callback(self, ros_data):
        '''Callback function of subscribed topic. 
        Here images get converted and features detected'''
        global find_counter, head_moved

        if (rospy.get_param('state') == 'normal' or rospy.get_param('state') == 'play'):
            #### direct conversion to CV2 ####
            np_arr = np.fromstring(ros_data.data, np.uint8)
            image_np = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)  # OpenCV >= 3.0:

            head_angle = Float64()

            greenLower = (50, 50, 20)
            greenUpper = (70, 255, 255)

            blurred = cv2.GaussianBlur(image_np, (11, 11), 0)
            hsv = cv2.cvtColor(blurred, cv2.COLOR_BGR2HSV)
            mask = cv2.inRange(hsv, greenLower, greenUpper)
            mask = cv2.erode(mask, None, iterations=2)
            mask = cv2.dilate(mask, None, iterations=2)
            cnts = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL,
                                    cv2.CHAIN_APPROX_SIMPLE)
            cnts = imutils.grab_contours(cnts)
            center = None
            # only proceed if at least one contour was found
            if len(cnts) > 0:
                # find the largest contour in the mask, then use
                # it to compute the minimum enclosing circle and
                # centroid
                if not rospy.get_param('state') == 'sleep':
                    rospy.set_param('ball_detected', 1)
                find_counter = 0
                c = max(cnts, key=cv2.contourArea)
                ((x, y), radius) = cv2.minEnclosingCircle(c)
                M = cv2.moments(c)
                center = (int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"]))
                self.ball_pub.publish(1)

                # only proceed if the radius meets a minimum size
                if radius > 105:
                    # draw the circle and centroid on the frame,
                    # then update the list of tracked points
                    cv2.circle(image_np, (int(x), int(y)), int(radius), (0, 255, 255), 2)
                    cv2.circle(image_np, center, 5, (0, 0, 255), -1) 
                    if rospy.get_param('state') == 'play':
                        vel = Twist()
                        vel.angular.z = -0.002 * (center[0] - 400)
                        vel.linear.x = -0.01 * (radius - 100)
                        self.vel_pub.publish(vel)
                elif (radius < 95 and rospy.get_param('state') == 'play'):
                    vel = Twist()
                    vel.angular.z = -0.002 * (center[0] - 400)
                    vel.linear.x = -0.01 * (radius - 100)
                    if vel.linear.x < 0.4:
                        vel.linear.x = 0.4
                    elif vel.linear.x > 1.0:
                        vel.linear.x = 1.0
                    if vel.angular.z < 0.1:
                        vel.angular.z = 0.1
                    elif vel.angular.z > 0.3:
                        vel.angular.z = 0.3
                    self.vel_pub.publish(vel)
                elif (rospy.get_param('state') == 'play' and radius <= 105 and radius >= 95):
                    if head_moved == 0:
                        rospy.loginfo('*The dog is turning the head...*')
                        head_angle = 0
                        vel = Twist()
                        vel.linear.x = 0
                        vel.angular.z = 0
                        self.vel_pub.publish(vel)
                        self.head_pub.publish(head_angle)
                        while head_angle < (0.7854):
                            head_angle = head_angle + 0.1
                            self.head_pub.publish(head_angle)
                            image_np = cv2.rotate(image_np, cv2.ROTATE_90_CLOCKWISE)
                            cv2.imshow('window', image_np)
                            time.sleep(1 / sim_scale)
                        time.sleep(1 / sim_scale)
                        while head_angle > (-0.7854):
                            head_angle = head_angle - 0.1
                            self.head_pub.publish(head_angle)
                            image_np = cv2.rotate(image_np, cv2.ROTATE_90_CLOCKWISE)
                            cv2.imshow('window', image_np)
                            time.sleep(1 / sim_scale)
                        time.sleep(1 / sim_scale)
                        while head_angle < 0:
                            head_angle = head_angle + 0.1
                            self.head_pub.publish(head_angle)
                            image_np = cv2.rotate(image_np, cv2.ROTATE_90_CLOCKWISE)
                            cv2.imshow('window', image_np)
                            time.sleep(1 / sim_scale)
                        head_angle = 0
                        self.head_pub.publish(head_angle)   
                        head_moved = 1              

            elif (rospy.get_param('state') == 'play' and find_counter < 100):
                head_moved = 0
                vel = Twist()
                vel.angular.z = 0.5
                self.vel_pub.publish(vel)
                find_counter = find_counter + 1
            elif (rospy.get_param('state') == 'play' and find_counter >= 100):
                head_moved = 0
                vel = Twist()
                vel.angular.z = 0
                self.vel_pub.publish(vel)
                self.ball_pub.publish(2)
                rospy.set_param('ball_detected', 0)

            image_np = cv2.rotate(image_np, cv2.ROTATE_90_CLOCKWISE)
            cv2.imshow('window', image_np)
            cv2.waitKey(2)

## Initializes the image_feature class and spins until interrupted by a keyboard command
def main(args):
    '''Initializes and cleanups ros node'''
    ic = image_feature()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print ("Shutting down ROS Image feature detector module")
    cv2.destroyAllWindows()


if __name__ == '__main__':
    main(sys.argv)
