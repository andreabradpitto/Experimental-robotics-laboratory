#!/usr/bin/env python

## @package dog_vision
# Implements a vision module for the robotic dog that uses OpenCV in order to constantly
# scan the surroundings, looking for specific colored balls.
# This node is able to take control, when needed, of the robot movements, allowing it
# to reach a room when a corresponding new ball is discovered. It also stores
# the positions of the balls discovered, thus learning the displacement of the rooms
# inside the house as time passes. A simple algorithm has been added to the script
# in order to unstick the robotic dog as the ball chasing behavior may cause a
# collision with other entities in the house

# Python library
import sys

# NumPy
import numpy as np

import random
import time

# OpenCV
import cv2

import imutils

# Ros library
import rospy

# Ros Messages
from sensor_msgs.msg import CompressedImage
from geometry_msgs.msg import Twist

import actionlib
from assignment3.srv import Explore
from nav_msgs.msg import Odometry
from move_base_msgs.msg import MoveBaseAction

## set lower bound of the BGR color coding for blue ball recognition
blueLower = (100, 50, 50)
## set upper bound of the BGR color coding for blue ball recognition
blueUpper = (130, 255, 255)
## set lower bound of the BGR color coding for red ball recognition
redLower = (0, 50, 50)
## set upper bound of the BGR color coding for red ball recognition
redUpper = (5, 255, 255)
## set lower bound of the BGR color coding for green ball recognition
greenLower = (50, 50, 50)
## set upper bound of the BGR color coding for green ball recognition
greenUpper = (70, 255, 255)
## set lower bound of the BGR color coding for yellow ball recognition
yellowLower = (25, 50, 50)
## set upper bound of the BGR color coding for yellow ball recognition
yellowUpper = (35, 255, 255)
## set lower bound of the BGR color coding for magenta ball recognition
magentaLower = (125, 50, 50)
## set upper bound of the BGR color coding for magenta ball recognition
magentaUpper = (150, 255, 255)
## set lower bound of the BGR color coding for black ball recognition
blackLower = (0, 0, 0)
## set upper bound of the BGR color coding for black ball recognition
blackUpper = (5, 50, 50)

## defines the current knowledge state of the coordinates corresponding to the blue ball
# (0 = not yet discovered; 1 = in progress; 2 = completed)
blue_solved = 0
## defines the current knowledge state of the coordinates corresponding to the red ball
# (0 = not yet discovered; 1 = in progress; 2 = completed)
red_solved = 0
## defines the current knowledge state of the coordinates corresponding to the green ball
# (0 = not yet discovered; 1 = in progress; 2 = completed)
green_solved = 0
## defines the current knowledge state of the coordinates corresponding to the yellow ball
# (0 = not yet discovered; 1 = in progress; 2 = completed)
yellow_solved = 0
## defines the current knowledge state of the coordinates corresponding to the magenta ball
# (0 = not yet discovered; 1 = in progress; 2 = completed)
magenta_solved = 0
## defines the current knowledge state of the coordinates corresponding to the black ball
# (0 = not yet discovered; 1 = in progress; 2 = completed)
black_solved = 0

## Acquire simulation speed scaling factor from launch file
sim_scale = rospy.get_param('sim_scale')

## variables used to try and trigger a predetermined procedure if the robot gets stuck
stuck_counter = 0
previous_radius = 0

## Constant determining how many callbacks to wait before trying to unstick the robot
STUCK_PATIENCE = 50

## Class used to visualize what the the robotic dog see during motion. It uses
# OpenCV in order to acquire images of the house, and takes control of the
# robot movements as soon as a new ball is spotted, with the aim of getting near it
# and storing the location for later reference
class image_feature:

    ## Class initialization: subscribe to topics and declare publishers
    def __init__(self):
        rospy.init_node('dog_vision_node', anonymous = True)

        ## topic used to publish robot velocity commands
        self.vel_pub = rospy.Publisher("cmd_vel", Twist, queue_size=1)

        ## subscribed topic, used to check if balls are present nearby
        self.subscriber = rospy.Subscriber("eye_camera/image_raw/compressed",
                                           CompressedImage, self.callback, queue_size=1)

    ## Function used to compute the amount of image contours comprised inside
    # a specific color's bounds
    def computeCount(self, hsv, colorLower, colorUpper):
            colorMask = cv2.inRange(hsv, colorLower, colorUpper)
            colorMask = cv2.erode(colorMask, None, iterations=2)
            colorMask = cv2.dilate(colorMask, None, iterations=2)
            colorCnts = cv2.findContours(colorMask.copy(), cv2.RETR_EXTERNAL,
                                    cv2.CHAIN_APPROX_SIMPLE)
            colorCnts = imutils.grab_contours(colorCnts)
            return colorCnts

    ## Callback of the subscribed topic, used to constantly show on screen what
    # the robotic dog is seeing. This callback also performs image conversion
    # and feature detection. It has a different algorithm depending on the robotic
    # dog current state. When the robot is in the Sleep or Play state, this callback
    # just provides visual feedback on screen about what the robot is currently seeing.
    # With this choice, the robot's Play state actually prioritizes the current human
    # request over any possible new ball detection, i.e. they are ignored.
    # When the robot is in Normal state, this callback does nothing if no balls are
    # detected. If this is not the case, the robot checks if that ball is present
    # in its database and, if so, it ignores it. If the ball preceived is a new discovery,
    # the callback cancels all current move_base goals through an action client, and
    # takes control of the dog motion: it leads the robot close to the ball,
    # and then stores its coordinates. While this process is being carried out,
    # the code draws both a colored dot centered on the ball and a circle outlining the
    # latter; the callback also locks the triggering of new ball discovery processes,
    # and frees them only once it is completed. The callback behavior for the Find state
    # is similar to the one of the Normal state, but this time the goal cancelling
    # is sent to the explore_lite algorithm via a service client. The code also
    # checks if the ball found and reached was actually the wanted one:
    # if so, the finding process is over, otherwise the callback restarts the
    # explore_lite algorithm, with another service client, in order to find
    # another new ball. The two service clients used to communicate with the Explore
    # class (which hosts the exploration algorithm) rely on as many service servers,
    # that I added as methods of the class of the original code. This callback is
    # particularly long code-wise, as I tried to make it as fast as possible, limiting
    # function calls and argument substitutions, at the cost of being a bit less readable
    def callback(self, ros_data):
        global blue_solved, red_solved, green_solved, \
               yellow_solved, magenta_solved, black_solved, \
               stuck_counter, previous_radius, map_x_min, map_x_max, \
               map_y_min, map_y_max, STUCK_PATIENCE

        ## Direct conversion to CV2
        np_arr = np.fromstring(ros_data.data, np.uint8)
        image_np = cv2.imdecode(np_arr, cv2.IMREAD_COLOR) # OpenCV >= 3.0:



        if (rospy.get_param('state') == 'normal'):

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
                ## action client used only to stop move_base execution
                mb_vision_client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
                mb_vision_client.cancel_all_goals()
                blue_solved = 1
                # find the largest contour in the blue mask, then use
                # it to compute the minimum enclosing circle and centroid
                c = max(blueCnts, key=cv2.contourArea)
                ((x, y), radius) = cv2.minEnclosingCircle(c)
                m_mat = cv2.moments(c)
                center = (int(m_mat["m10"] / m_mat["m00"]), int(m_mat["m01"] / m_mat["m00"]))

                # only proceed if the radius meets a minimum size
                if(center[0] < 396 or center[0] > 404 or radius < 99 or radius > 101):
                    # draw the circle and centroid on the frame,
                    # then update the list of tracked points
                    if previous_radius == radius:
                        stuck_counter = stuck_counter + 1
                    previous_radius = radius
                    # draw a yellow circle
                    cv2.circle(image_np, (int(x), int(y)), int(radius), (0, 255, 255), 2)
                    # draw a red dot
                    cv2.circle(image_np, center, 5, (0, 0, 255), -1)
                    vel = Twist()
                    vel.angular.z = -0.002 * (center[0] - 400)
                    vel.linear.x = -0.01 * (radius - 100)
                    if(vel.linear.x > 0.4):
                        vel.linear.x = 0.4
                    elif(vel.linear.x < -0.4):
                        vel.linear.x = -0.4
                    self.vel_pub.publish(vel)
                    # try to free the robot
                    if stuck_counter > STUCK_PATIENCE:
                        rospy.set_param('stuck', 1)
                        rospy.set_param('new_ball_detected', 0)
                        rospy.loginfo('Dog: I am stuck! I will try to free myself' \
                                      ' (it might take a few attempts...)')
                        vel.linear.x = -0.4
                        vel.angular.z = random.randint(-4, 4) / 10.0
                        self.vel_pub.publish(vel)
                        time.sleep(5 / sim_scale)
                        blue_solved = 0
                        stuck_counter = 0
                else:
                    # the robot reached the ball: store coordinates
                    # corresponding to its color
                    pos = rospy.wait_for_message('odom', Odometry, timeout = None)
                    rospy.set_param('blue/x', pos.pose.pose.position.x)
                    rospy.set_param('blue/y', pos.pose.pose.position.y)
                    rospy.loginfo('Dog: I have stored the entrance position (' \
                                  + tcolors.BLUE + 'blue ball' + tcolors.ENDC + ')')
                    blue_solved = 2
                    rospy.set_param('new_ball_detected', 0)
                    stuck_counter = 0

            elif(len(redCnts) > 0 and red_solved != 2 \
                 and blue_solved != 1 and green_solved != 1 and yellow_solved != 1 \
                 and magenta_solved != 1 and black_solved != 1):
                rospy.set_param('new_ball_detected', 1)
                ## action client used only to stop move_base execution
                mb_vision_client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
                mb_vision_client.cancel_all_goals()
                red_solved = 1
                # find the largest contour in the red mask, then use
                # it to compute the minimum enclosing circle centroid
                c = max(redCnts, key=cv2.contourArea)
                ((x, y), radius) = cv2.minEnclosingCircle(c)
                m_mat = cv2.moments(c)
                center = (int(m_mat["m10"] / m_mat["m00"]), int(m_mat["m01"] / m_mat["m00"]))

                # only proceed if the radius meets a minimum size
                if(center[0] < 396 or center[0] > 404 or radius < 99 or radius > 101):
                    # draw the circle and centroid on the frame,
                    # then update the list of tracked points
                    if previous_radius == radius:
                        stuck_counter = stuck_counter + 1
                    previous_radius = radius
                    # draw a yellow circle
                    cv2.circle(image_np, (int(x), int(y)), int(radius), (0, 255, 255), 2)
                    # draw a green dot
                    cv2.circle(image_np, center, 5, (0, 255, 0), -1)
                    vel = Twist()
                    vel.angular.z = -0.002 * (center[0] - 400)
                    vel.linear.x = -0.01 * (radius - 100)
                    if(vel.linear.x > 0.4):
                        vel.linear.x = 0.4
                    elif(vel.linear.x < -0.4):
                        vel.linear.x = -0.4
                    self.vel_pub.publish(vel)
                    # try to free the robot
                    if stuck_counter > STUCK_PATIENCE:
                        rospy.set_param('stuck', 1)
                        rospy.set_param('new_ball_detected', 0)
                        rospy.loginfo('Dog: I am stuck! I will try to free myself' \
                                      ' (it might take a few attempts...)')
                        vel.linear.x = -0.4
                        vel.angular.z = random.randint(-4, 4) / 10.0
                        self.vel_pub.publish(vel)
                        time.sleep(5 / sim_scale)
                        red_solved = 0
                        stuck_counter = 0
                else:
                    # the robot reached the ball: store coordinates
                    # corresponding to its color
                    pos = rospy.wait_for_message('odom', Odometry, timeout = None)
                    rospy.set_param('red/x', pos.pose.pose.position.x)
                    rospy.set_param('red/y', pos.pose.pose.position.y)
                    rospy.loginfo('Dog: I have stored the closet position (' \
                                  + tcolors.RED + 'red ball' + tcolors.ENDC + ')')
                    red_solved = 2
                    rospy.set_param('new_ball_detected', 0)
                    stuck_counter = 0

            elif(len(greenCnts) > 0 and green_solved != 2 \
                 and blue_solved != 1 and red_solved != 1 and yellow_solved != 1 \
                 and magenta_solved != 1 and black_solved != 1):
                rospy.set_param('new_ball_detected', 1)
                ## action client used only to stop move_base execution
                mb_vision_client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
                mb_vision_client.cancel_all_goals()
                green_solved = 1
                # find the largest contour in the green mask, then use
                # it to compute the minimum enclosing circle and centroid
                c = max(greenCnts, key=cv2.contourArea)
                ((x, y), radius) = cv2.minEnclosingCircle(c)
                m_mat = cv2.moments(c)
                center = (int(m_mat["m10"] / m_mat["m00"]), int(m_mat["m01"] / m_mat["m00"]))

                # only proceed if the radius meets a minimum size
                if(center[0] < 396 or center[0] > 404 or radius < 99 or radius > 101):
                    # draw the circle and centroid on the frame,
                    # then update the list of tracked points
                    if previous_radius == radius:
                        stuck_counter = stuck_counter + 1
                    previous_radius = radius
                    # draw a yellow circle
                    cv2.circle(image_np, (int(x), int(y)), int(radius), (0, 255, 255), 2)
                    # draw a red dot
                    cv2.circle(image_np, center, 5, (0, 0, 255), -1)
                    vel = Twist()
                    vel.angular.z = -0.002 * (center[0] - 400)
                    vel.linear.x = -0.01 * (radius - 100)
                    if(vel.linear.x > 0.4):
                        vel.linear.x = 0.4
                    elif(vel.linear.x < -0.4):
                        vel.linear.x = -0.4
                    self.vel_pub.publish(vel)
                    # try to free the robot
                    if stuck_counter > STUCK_PATIENCE:
                        rospy.set_param('stuck', 1)
                        rospy.set_param('new_ball_detected', 0)
                        rospy.loginfo('Dog: I am stuck! I will try to free myself' \
                                      ' (it might take a few attempts...)')
                        vel.linear.x = -0.4
                        vel.angular.z = random.randint(-4, 4) / 10.0
                        self.vel_pub.publish(vel)
                        time.sleep(5 / sim_scale)
                        green_solved = 0
                        stuck_counter = 0
                else:
                    # the robot reached the ball: store coordinates
                    # corresponding to its color
                    pos = rospy.wait_for_message('odom', Odometry, timeout = None)
                    rospy.set_param('green/x', pos.pose.pose.position.x)
                    rospy.set_param('green/y', pos.pose.pose.position.y)
                    rospy.loginfo('Dog: I have stored the livingroom position (' \
                                  + tcolors.GREEN + 'green ball' + tcolors.ENDC + ')')
                    green_solved = 2
                    rospy.set_param('new_ball_detected', 0)
                    stuck_counter = 0

            elif(len(yellowCnts) > 0 and yellow_solved != 2 \
                 and blue_solved != 1 and red_solved != 1 and green_solved != 1
                 and magenta_solved != 1 and black_solved != 1):
                rospy.set_param('new_ball_detected', 1)
                ## action client used only to stop move_base execution
                mb_vision_client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
                mb_vision_client.cancel_all_goals()
                yellow_solved = 1
                # find the largest contour in the yellow mask, then use
                # it to compute the minimum enclosing circle and centroid
                c = max(yellowCnts, key=cv2.contourArea)
                ((x, y), radius) = cv2.minEnclosingCircle(c)
                m_mat = cv2.moments(c)
                center = (int(m_mat["m10"] / m_mat["m00"]), int(m_mat["m01"] / m_mat["m00"]))

                # only proceed if the radius meets a minimum size
                if(center[0] < 396 or center[0] > 404 or radius < 99 or radius > 101):
                    # draw the circle and centroid on the frame,
                    # then update the list of tracked points
                    if previous_radius == radius:
                        stuck_counter = stuck_counter + 1
                    previous_radius = radius
                    # draw a magenta circle
                    cv2.circle(image_np, (int(x), int(y)), int(radius), (255, 0, 255), 2)
                    # draw a red dot
                    cv2.circle(image_np, center, 5, (0, 0, 255), -1)
                    vel = Twist()
                    vel.angular.z = -0.002 * (center[0] - 400)
                    vel.linear.x = -0.01 * (radius - 100)
                    if(vel.linear.x > 0.4):
                        vel.linear.x = 0.4
                    elif(vel.linear.x < -0.4):
                        vel.linear.x = -0.4
                    self.vel_pub.publish(vel)
                    # try to free the robot
                    if stuck_counter > STUCK_PATIENCE:
                        rospy.set_param('stuck', 1)
                        rospy.set_param('new_ball_detected', 0)
                        rospy.loginfo('Dog: I am stuck! I will try to free myself' \
                                      ' (it might take a few attempts...)')
                        vel.linear.x = -0.4
                        vel.angular.z = random.randint(-4, 4) / 10.0
                        self.vel_pub.publish(vel)
                        time.sleep(5 / sim_scale)
                        yellow_solved = 0
                        stuck_counter = 0
                else:
                    # the robot reached the ball: store coordinates
                    # corresponding to its color
                    pos = rospy.wait_for_message('odom', Odometry, timeout = None)
                    rospy.set_param('yellow/x', pos.pose.pose.position.x)
                    rospy.set_param('yellow/y', pos.pose.pose.position.y)
                    rospy.loginfo('Dog: I have stored the kitchen position (' \
                                  + tcolors.YELLOW + 'yellow ball' + tcolors.ENDC + ')')
                    yellow_solved = 2
                    rospy.set_param('new_ball_detected', 0)
                    stuck_counter = 0

            elif(len(magentaCnts) > 0 and magenta_solved != 2 \
                 and blue_solved != 1 and red_solved != 1 and green_solved != 1
                 and yellow_solved != 1 and black_solved != 1):
                rospy.set_param('new_ball_detected', 1)
                ## action client used only to stop move_base execution
                mb_vision_client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
                mb_vision_client.cancel_all_goals()
                magenta_solved = 1
                # find the largest contour in the magenta mask, then use
                # it to compute the minimum enclosing circle and centroid
                c = max(magentaCnts, key=cv2.contourArea)
                ((x, y), radius) = cv2.minEnclosingCircle(c)
                m_mat = cv2.moments(c)
                center = (int(m_mat["m10"] / m_mat["m00"]), int(m_mat["m01"] / m_mat["m00"]))

                # only proceed if the radius meets a minimum size
                if(center[0] < 396 or center[0] > 404 or radius < 99 or radius > 101):
                    # draw the circle and centroid on the frame,
                    # then update the list of tracked points
                    if previous_radius == radius:
                        stuck_counter = stuck_counter + 1
                    previous_radius = radius
                    # draw a yellow circle
                    cv2.circle(image_np, (int(x), int(y)), int(radius), (0, 255, 255), 2)
                    # draw a green dot
                    cv2.circle(image_np, center, 5, (0, 255, 0), -1)
                    vel = Twist()
                    vel.angular.z = -0.002 * (center[0] - 400)
                    vel.linear.x = -0.01 * (radius - 100)
                    if(vel.linear.x > 0.4):
                        vel.linear.x = 0.4
                    elif(vel.linear.x < -0.4):
                        vel.linear.x = -0.4
                    self.vel_pub.publish(vel)
                    # try to free the robot
                    if stuck_counter > STUCK_PATIENCE:
                        rospy.set_param('stuck', 1)
                        rospy.set_param('new_ball_detected', 0)
                        rospy.loginfo('Dog: I am stuck! I will try to free myself' \
                                      ' (it might take a few attempts...)')
                        vel.linear.x = -0.4
                        vel.angular.z = random.randint(-4, 4) / 10.0
                        self.vel_pub.publish(vel)
                        time.sleep(5 / sim_scale)
                        magenta_solved = 0
                        stuck_counter = 0
                else:
                    # the robot reached the ball: store coordinates
                    # corresponding to its color
                    pos = rospy.wait_for_message('odom', Odometry, timeout = None)
                    rospy.set_param('magenta/x', pos.pose.pose.position.x)
                    rospy.set_param('magenta/y', pos.pose.pose.position.y)
                    rospy.loginfo('Dog: I have stored the bathroom position (' \
                                  + tcolors.MAGENTA + 'magenta ball' + tcolors.ENDC + ')')
                    magenta_solved = 2
                    rospy.set_param('new_ball_detected', 0)
                    stuck_counter = 0

            elif(len(blackCnts) > 0 and black_solved != 2 \
                 and blue_solved != 1 and red_solved != 1 and green_solved != 1
                 and yellow_solved != 1 and magenta_solved != 1):
                rospy.set_param('new_ball_detected', 1)
                ## action client used only to stop move_base execution
                mb_vision_client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
                mb_vision_client.cancel_all_goals()
                black_solved = 1
                # find the largest contour in the black mask, then use
                # it to compute the minimum enclosing circle and centroid
                c = max(blackCnts, key=cv2.contourArea)
                ((x, y), radius) = cv2.minEnclosingCircle(c)
                m_mat = cv2.moments(c)
                center = (int(m_mat["m10"] / m_mat["m00"]), int(m_mat["m01"] / m_mat["m00"]))

                # only proceed if the radius meets a minimum size
                if(center[0] < 396 or center[0] > 404 or radius < 99 or radius > 101):
                    # draw the circle and centroid on the frame,
                    # then update the list of tracked points
                    if previous_radius == radius:
                        stuck_counter = stuck_counter + 1
                    previous_radius = radius
                    # draw a yellow circle
                    cv2.circle(image_np, (int(x), int(y)), int(radius), (0, 255, 255), 2)
                    # draw a red dot
                    cv2.circle(image_np, center, 5, (0, 0, 255), -1)
                    vel = Twist()
                    vel.angular.z = -0.002 * (center[0] - 400)
                    vel.linear.x = -0.01 * (radius - 100)
                    if(vel.linear.x > 0.4):
                        vel.linear.x = 0.4
                    elif(vel.linear.x < -0.4):
                        vel.linear.x = -0.4
                    self.vel_pub.publish(vel)
                    # try to free the robot
                    if stuck_counter > STUCK_PATIENCE:
                        rospy.set_param('stuck', 1)
                        rospy.set_param('new_ball_detected', 0)
                        rospy.loginfo('Dog: I am stuck! I will try to free myself' \
                                      ' (it might take a few attempts...)')
                        vel.linear.x = -0.4
                        vel.angular.z = random.randint(-4, 4) / 10.0
                        self.vel_pub.publish(vel)
                        time.sleep(5 / sim_scale)
                        black_solved = 0
                        stuck_counter = 0
                else:
                    # the robot reached the ball: store coordinates
                    # corresponding to its color
                    pos = rospy.wait_for_message('odom', Odometry, timeout = None)
                    rospy.set_param('black/x', pos.pose.pose.position.x)
                    rospy.set_param('black/y', pos.pose.pose.position.y)
                    rospy.loginfo('Dog: I have stored the bedroom position (' \
                                  + tcolors.WHITE + 'black ball' + tcolors.ENDC + ')')
                    black_solved = 2
                    rospy.set_param('new_ball_detected', 0)
                    stuck_counter = 0

            elif(rospy.get_param('new_ball_detected', 1)):
                stuck_counter = stuck_counter + 1
                if (stuck_counter > STUCK_PATIENCE):
                    rospy.loginfo('Dog: I have spotted a ball through a wall! Well, ' \
                                  'this should not have happened...')
                    rospy.set_param('new_ball_detected', 0)
                    rospy.set_param('stuck', 1)
                    stuck_counter = 0
                    if(blue_solved == 1):
                        blue_solved = 0
                    if(red_solved == 1):
                        red_solved = 0
                    if(green_solved == 1):
                        green_solved = 0
                    if(yellow_solved == 1):
                        yellow_solved = 0
                    if(magenta_solved == 1):
                        magenta_solved = 0
                    if(black_solved == 1):
                        black_solved = 0


        elif(rospy.get_param('state') == 'find'):

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
                ## explore lite service client used to
                # stop explore_lite algorithm execution
                rospy.wait_for_service('explore_stop_service')
                explore_stop = rospy.ServiceProxy('explore_stop_service', Explore)
                explore_stop(0)
                blue_solved = 1
                # find the largest contour in the blue mask, then use
                # it to compute the minimum enclosing circle and centroid
                c = max(blueCnts, key=cv2.contourArea)
                ((x, y), radius) = cv2.minEnclosingCircle(c)
                m_mat = cv2.moments(c)
                center = (int(m_mat["m10"] / m_mat["m00"]), int(m_mat["m01"] / m_mat["m00"]))

                # only proceed if the radius meets a minimum size
                if(center[0] < 396 or center[0] > 404 or radius < 99 or radius > 101):
                    # draw the circle and centroid on the frame,
                    # then update the list of tracked points
                    if previous_radius == radius:
                        stuck_counter = stuck_counter + 1
                    previous_radius = radius
                    # draw a yellow circle
                    cv2.circle(image_np, (int(x), int(y)), int(radius), (0, 255, 255), 2)
                    # draw a red dot
                    cv2.circle(image_np, center, 5, (0, 0, 255), -1)
                    vel = Twist()
                    vel.angular.z = -0.002 * (center[0] - 400)
                    vel.linear.x = -0.01 * (radius - 100)
                    if(vel.linear.x > 0.4):
                        vel.linear.x = 0.4
                    elif(vel.linear.x < -0.4):
                        vel.linear.x = -0.4
                    self.vel_pub.publish(vel)
                    # try to free the robot
                    if stuck_counter > STUCK_PATIENCE:
                        rospy.set_param('stuck', 1)
                        rospy.set_param('new_ball_detected', 0)
                        rospy.loginfo('Dog: I am stuck! I will try to free myself' \
                                      ' (it might take a few attempts...)')
                        vel.linear.x = -0.4
                        vel.angular.z = random.randint(-4, 4) / 10.0
                        self.vel_pub.publish(vel)
                        time.sleep(5 / sim_scale)
                        blue_solved = 0
                        stuck_counter = 0
                else:
                    # the robot reached the ball: store coordinates
                    # corresponding to its color
                    pos = rospy.wait_for_message('odom', Odometry, timeout = None)
                    rospy.set_param('blue/x', pos.pose.pose.position.x)
                    rospy.set_param('blue/y', pos.pose.pose.position.y)
                    rospy.loginfo('Dog: I have stored the entrance position (' \
                                  + tcolors.BLUE + 'blue ball' + tcolors.ENDC + ')')
                    blue_solved = 2
                    vel = Twist()
                    vel.linear.x = 0
                    vel.angular.z = 0
                    self.vel_pub.publish(vel)
                    rospy.set_param('new_ball_detected', 0)
                    if rospy.get_param('unknown_ball') == 'blue':
                        # this is the ball the robot had to find
                        rospy.set_param('unknown_ball', 'none')
                    else:
                        ## explore_lite service client that lets the algorithm start
                        # exploring the robotic dog's surroundings
                        rospy.wait_for_service('explore_start_service')
                        explore_start = rospy.ServiceProxy('explore_start_service', Explore)
                        explore_start(1)
                    stuck_counter = 0

            elif(len(redCnts) > 0 and red_solved != 2 \
                 and blue_solved != 1 and green_solved != 1 and yellow_solved != 1
                 and magenta_solved != 1 and black_solved != 1):
                rospy.set_param('new_ball_detected', 1)
                ## explore lite service client used to
                # stop explore_lite algorithm execution
                rospy.wait_for_service('explore_stop_service')
                explore_stop = rospy.ServiceProxy('explore_stop_service', Explore)
                explore_stop(0)
                red_solved = 1
                # find the largest contour in the red mask, then use
                # it to compute the minimum enclosing circle and centroid
                c = max(redCnts, key=cv2.contourArea)
                ((x, y), radius) = cv2.minEnclosingCircle(c)
                m_mat = cv2.moments(c)
                center = (int(m_mat["m10"] / m_mat["m00"]), int(m_mat["m01"] / m_mat["m00"]))

                # only proceed if the radius meets a minimum size
                if(center[0] < 396 or center[0] > 404 or radius < 99 or radius > 101):
                    # draw the circle and centroid on the frame,
                    # then update the list of tracked points
                    if previous_radius == radius:
                        stuck_counter = stuck_counter + 1
                    previous_radius = radius
                    # draw a yellow circle
                    cv2.circle(image_np, (int(x), int(y)), int(radius), (0, 255, 255), 2)
                    # draw a green dot
                    cv2.circle(image_np, center, 5, (0, 255, 0), -1)
                    vel = Twist()
                    vel.angular.z = -0.002 * (center[0] - 400)
                    vel.linear.x = -0.01 * (radius - 100)
                    if(vel.linear.x > 0.4):
                        vel.linear.x = 0.4
                    elif(vel.linear.x < -0.4):
                        vel.linear.x = -0.4
                    self.vel_pub.publish(vel)
                    # try to free the robot
                    if stuck_counter > STUCK_PATIENCE:
                        rospy.set_param('stuck', 1)
                        rospy.set_param('new_ball_detected', 0)
                        rospy.loginfo('Dog: I am stuck! I will try to free myself' \
                                      ' (it might take a few attempts...)')
                        vel.linear.x = -0.4
                        vel.angular.z = random.randint(-4, 4) / 10.0
                        self.vel_pub.publish(vel)
                        time.sleep(5 / sim_scale)
                        red_solved = 0
                        stuck_counter = 0
                else:
                    # the robot reached the ball: store coordinates
                    # corresponding to its color
                    pos = rospy.wait_for_message('odom', Odometry, timeout = None)
                    rospy.set_param('red/x', pos.pose.pose.position.x)
                    rospy.set_param('red/y', pos.pose.pose.position.y)
                    rospy.loginfo('Dog: I have stored the closet position (' \
                                  + tcolors.RED + 'red ball' + tcolors.ENDC + ')')
                    red_solved = 2
                    vel = Twist()
                    vel.linear.x = 0
                    vel.angular.z = 0
                    self.vel_pub.publish(vel)
                    rospy.set_param('new_ball_detected', 0)
                    if rospy.get_param('unknown_ball') == 'red':
                        # this is the ball the robot had to find
                        rospy.set_param('unknown_ball', 'none')
                    else:
                        ## explore_lite service client that lets the algorithm start
                        # exploring the robotic dog's surroundings
                        rospy.wait_for_service('explore_start_service')
                        explore_start = rospy.ServiceProxy('explore_start_service', Explore)
                        explore_start(1)
                    stuck_counter = 0

            elif(len(greenCnts) > 0 and green_solved != 2 \
                 and blue_solved != 1 and red_solved != 1 and yellow_solved != 1 \
                 and magenta_solved != 1 and black_solved != 1):
                rospy.set_param('new_ball_detected', 1)
                ## explore lite service client used to
                # stop explore_lite algorithm execution
                rospy.wait_for_service('explore_stop_service')
                explore_stop = rospy.ServiceProxy('explore_stop_service', Explore)
                explore_stop(0)
                green_solved = 1
                # find the largest contour in the green mask, then use
                # it to compute the minimum enclosing circle and centroid
                c = max(greenCnts, key=cv2.contourArea)
                ((x, y), radius) = cv2.minEnclosingCircle(c)
                m_mat = cv2.moments(c)
                center = (int(m_mat["m10"] / m_mat["m00"]), int(m_mat["m01"] / m_mat["m00"]))

                # only proceed if the radius meets a minimum size
                if(center[0] < 396 or center[0] > 404 or radius < 99 or radius > 101):
                    # draw the circle and centroid on the frame,
                    # then update the list of tracked points
                    if previous_radius == radius:
                        stuck_counter = stuck_counter + 1
                    previous_radius = radius
                    # draw a yellow circle
                    cv2.circle(image_np, (int(x), int(y)), int(radius), (0, 255, 255), 2)
                    # draw a red dot
                    cv2.circle(image_np, center, 5, (0, 0, 255), -1)
                    vel = Twist()
                    vel.angular.z = -0.002 * (center[0] - 400)
                    vel.linear.x = -0.01 * (radius - 100)
                    if(vel.linear.x > 0.4):
                        vel.linear.x = 0.4
                    elif(vel.linear.x < -0.4):
                        vel.linear.x = -0.4
                    self.vel_pub.publish(vel)
                    # try to free the robot
                    if stuck_counter > STUCK_PATIENCE:
                        rospy.set_param('stuck', 1)
                        rospy.set_param('new_ball_detected', 0)
                        rospy.loginfo('Dog: I am stuck! I will try to free myself' \
                                      ' (it might take a few attempts...)')
                        vel.linear.x = -0.4
                        vel.angular.z = random.randint(-4, 4) / 10.0
                        self.vel_pub.publish(vel)
                        time.sleep(5 / sim_scale)
                        green_solved = 0
                        stuck_counter = 0
                else:
                    # the robot reached the ball: store coordinates
                    # corresponding to its color
                    pos = rospy.wait_for_message('odom', Odometry, timeout = None)
                    rospy.set_param('green/x', pos.pose.pose.position.x)
                    rospy.set_param('green/y', pos.pose.pose.position.y)
                    rospy.loginfo('Dog: I have stored the livingroom position (' \
                                  + tcolors.GREEN + 'green ball' + tcolors.ENDC + ')')
                    green_solved = 2
                    vel = Twist()
                    vel.linear.x = 0
                    vel.angular.z = 0
                    self.vel_pub.publish(vel)
                    rospy.set_param('new_ball_detected', 0)
                    if rospy.get_param('unknown_ball') == 'green':
                        # this is the ball the robot had to find
                        rospy.set_param('unknown_ball', 'none')
                    else:
                        ## explore_lite service client that lets the algorithm start
                        # exploring the robotic dog's surroundings
                        rospy.wait_for_service('explore_start_service')
                        explore_start = rospy.ServiceProxy('explore_start_service', Explore)
                        explore_start(1)
                    stuck_counter = 0

            elif(len(yellowCnts) > 0 and yellow_solved != 2 \
                 and blue_solved != 1 and red_solved != 1 and green_solved != 1 \
                 and magenta_solved != 1 and black_solved != 1):
                rospy.set_param('new_ball_detected', 1)
                ## explore lite service client used to
                # stop explore_lite algorithm execution
                rospy.wait_for_service('explore_stop_service')
                explore_stop = rospy.ServiceProxy('explore_stop_service', Explore)
                explore_stop(0)
                yellow_solved = 1
                # find the largest contour in the yellow mask, then use
                # it to compute the minimum enclosing circle and centroid
                c = max(yellowCnts, key=cv2.contourArea)
                ((x, y), radius) = cv2.minEnclosingCircle(c)
                m_mat = cv2.moments(c)
                center = (int(m_mat["m10"] / m_mat["m00"]), int(m_mat["m01"] / m_mat["m00"]))

                # only proceed if the radius meets a minimum size
                if(center[0] < 396 or center[0] > 404 or radius < 99 or radius > 101):
                    # draw the circle and centroid on the frame,
                    # then update the list of tracked points
                    if previous_radius == radius:
                        stuck_counter = stuck_counter + 1
                    previous_radius = radius
                    # draw a magenta circle
                    cv2.circle(image_np, (int(x), int(y)), int(radius), (255, 0, 255), 2)
                    # draw a red dot
                    cv2.circle(image_np, center, 5, (0, 0, 255), -1)
                    vel = Twist()
                    vel.angular.z = -0.002 * (center[0] - 400)
                    vel.linear.x = -0.01 * (radius - 100)
                    if(vel.linear.x > 0.4):
                        vel.linear.x = 0.4
                    elif(vel.linear.x < -0.4):
                        vel.linear.x = -0.4
                    self.vel_pub.publish(vel)
                    # try to free the robot
                    if stuck_counter > STUCK_PATIENCE:
                        rospy.set_param('stuck', 1)
                        rospy.set_param('new_ball_detected', 0)
                        rospy.loginfo('Dog: I am stuck! I will try to free myself' \
                                      ' (it might take a few attempts...)')
                        vel.linear.x = -0.4
                        vel.angular.z = random.randint(-4, 4) / 10.0
                        self.vel_pub.publish(vel)
                        time.sleep(5 / sim_scale)
                        yellow_solved = 0
                        stuck_counter = 0
                else:
                    # the robot reached the ball: store coordinates
                    # corresponding to its color
                    pos = rospy.wait_for_message('odom', Odometry, timeout = None)
                    rospy.set_param('yellow/x', pos.pose.pose.position.x)
                    rospy.set_param('yellow/y', pos.pose.pose.position.y)
                    rospy.loginfo('Dog: I have stored the kitchen position (' \
                                  + tcolors.YELLOW + 'yellow ball' + tcolors.ENDC + ')')
                    yellow_solved = 2
                    vel = Twist()
                    vel.linear.x = 0
                    vel.angular.z = 0
                    self.vel_pub.publish(vel)
                    rospy.set_param('new_ball_detected', 0)
                    if rospy.get_param('unknown_ball') == 'yellow':
                        # this is the ball the robot had to find
                        rospy.set_param('unknown_ball', 'none')
                    else:
                        ## explore_lite service client that lets the algorithm start
                        # exploring the robotic dog's surroundings
                        rospy.wait_for_service('explore_start_service')
                        explore_start = rospy.ServiceProxy('explore_start_service', Explore)
                        explore_start(1)
                    stuck_counter = 0

            elif(len(magentaCnts) > 0 and magenta_solved != 2 \
                 and blue_solved != 1 and red_solved != 1 and green_solved != 1 \
                 and yellow_solved != 1 and black_solved != 1):
                rospy.set_param('new_ball_detected', 1)
                ## explore lite service client used to
                # stop explore_lite algorithm execution
                rospy.wait_for_service('explore_stop_service')
                explore_stop = rospy.ServiceProxy('explore_stop_service', Explore)
                explore_stop(0)
                magenta_solved = 1
                # find the largest contour in the magenta mask, then use
                # it to compute the minimum enclosing circle and centroid
                c = max(magentaCnts, key=cv2.contourArea)
                ((x, y), radius) = cv2.minEnclosingCircle(c)
                m_mat = cv2.moments(c)
                center = (int(m_mat["m10"] / m_mat["m00"]), int(m_mat["m01"] / m_mat["m00"]))

                # only proceed if the radius meets a minimum size
                if(center[0] < 396 or center[0] > 404 or radius < 99 or radius > 101):
                    # draw the circle and centroid on the frame,
                    # then update the list of tracked points
                    if previous_radius == radius:
                        stuck_counter = stuck_counter + 1
                    previous_radius = radius
                    # draw a yellow circle
                    cv2.circle(image_np, (int(x), int(y)), int(radius), (0, 255, 255), 2)
                    # draw a green dot
                    cv2.circle(image_np, center, 5, (0, 255, 0), -1)
                    vel = Twist()
                    vel.angular.z = -0.002 * (center[0] - 400)
                    vel.linear.x = -0.01 * (radius - 100)
                    if(vel.linear.x > 0.4):
                        vel.linear.x = 0.4
                    elif(vel.linear.x < -0.4):
                        vel.linear.x = -0.4
                    self.vel_pub.publish(vel)
                    # try to free the robot
                    if stuck_counter > STUCK_PATIENCE:
                        rospy.set_param('stuck', 1)
                        rospy.set_param('new_ball_detected', 0)
                        rospy.loginfo('Dog: I am stuck! I will try to free myself' \
                                      ' (it might take a few attempts...)')
                        vel.linear.x = -0.4
                        vel.angular.z = random.randint(-4, 4) / 10.0
                        self.vel_pub.publish(vel)
                        time.sleep(5 / sim_scale)
                        magenta_solved = 0
                        stuck_counter = 0
                else:
                    # the robot reached the ball: store coordinates
                    # corresponding to its color
                    pos = rospy.wait_for_message('odom', Odometry, timeout = None)
                    rospy.set_param('magenta/x', pos.pose.pose.position.x)
                    rospy.set_param('magenta/y', pos.pose.pose.position.y)
                    rospy.loginfo('Dog: I have stored the bathroom position (' \
                                  + tcolors.MAGENTA + 'magenta ball' + tcolors.ENDC + ')')
                    magenta_solved = 2
                    vel = Twist()
                    vel.linear.x = 0
                    vel.angular.z = 0
                    self.vel_pub.publish(vel)
                    rospy.set_param('new_ball_detected', 0)
                    if rospy.get_param('unknown_ball') == 'magenta':
                        # this is the ball the robot had to find
                        rospy.set_param('unknown_ball', 'none')
                    else:
                        ## explore_lite service client that lets the algorithm start
                        # exploring the robotic dog's surroundings
                        rospy.wait_for_service('explore_start_service')
                        explore_start = rospy.ServiceProxy('explore_start_service', Explore)
                        explore_start(1)
                    stuck_counter = 0

            elif(len(blackCnts) > 0 and black_solved != 2 \
                 and blue_solved != 1 and red_solved != 1 and green_solved != 1 \
                 and yellow_solved != 1 and magenta_solved != 1):
                rospy.set_param('new_ball_detected', 1)
                ## explore lite service client used to
                # stop explore_lite algorithm execution
                rospy.wait_for_service('explore_stop_service')
                explore_stop = rospy.ServiceProxy('explore_stop_service', Explore)
                explore_stop(0)
                black_solved = 1
                # find the largest contour in the black mask, then use
                # it to compute the minimum enclosing circle and centroid
                c = max(blackCnts, key=cv2.contourArea)
                ((x, y), radius) = cv2.minEnclosingCircle(c)
                m_mat = cv2.moments(c)
                center = (int(m_mat["m10"] / m_mat["m00"]), int(m_mat["m01"] / m_mat["m00"]))

                # only proceed if the radius meets a minimum size
                if(center[0] < 396 or center[0] > 404 or radius < 99 or radius > 101):
                    # draw the circle and centroid on the frame,
                    # then update the list of tracked points
                    if previous_radius == radius:
                        stuck_counter = stuck_counter + 1
                    previous_radius = radius
                    # draw a yellow circle
                    cv2.circle(image_np, (int(x), int(y)), int(radius), (0, 255, 255), 2)
                    # draw a red dot
                    cv2.circle(image_np, center, 5, (0, 0, 255), -1)
                    vel = Twist()
                    vel.angular.z = -0.002 * (center[0] - 400)
                    vel.linear.x = -0.01 * (radius - 100)
                    if(vel.linear.x > 0.4):
                        vel.linear.x = 0.4
                    elif(vel.linear.x < -0.4):
                        vel.linear.x = -0.4
                    self.vel_pub.publish(vel)
                    # try to free the robot
                    if stuck_counter > STUCK_PATIENCE:
                        rospy.set_param('stuck', 1)
                        rospy.set_param('new_ball_detected', 0)
                        rospy.loginfo('Dog: I am stuck! I will try to free myself' \
                                      ' (it might take a few attempts...)')
                        vel.linear.x = -0.4
                        vel.angular.z = random.randint(-4, 4) / 10.0
                        self.vel_pub.publish(vel)
                        time.sleep(5 / sim_scale)
                        black_solved = 0
                        stuck_counter = 0
                else:
                    # the robot reached the ball: store coordinates
                    # corresponding to its color
                    pos = rospy.wait_for_message('odom', Odometry, timeout = None)
                    rospy.set_param('black/x', pos.pose.pose.position.x)
                    rospy.set_param('black/y', pos.pose.pose.position.y)
                    rospy.loginfo('Dog: I have stored the bedroom position (' \
                                  + tcolors.WHITE + 'black ball' + tcolors.ENDC + ')')
                    black_solved = 2
                    vel = Twist()
                    vel.linear.x = 0
                    vel.angular.z = 0
                    self.vel_pub.publish(vel)
                    rospy.set_param('new_ball_detected', 0)
                    if rospy.get_param('unknown_ball') == 'black':
                        # this is the ball the robot had to find
                        rospy.set_param('unknown_ball', 'none')
                    else:
                        ## explore_lite service client that lets the algorithm start
                        # exploring the robotic dog's surroundings
                        rospy.wait_for_service('explore_start_service')
                        explore_start = rospy.ServiceProxy('explore_start_service', Explore)
                        explore_start(1)
                    stuck_counter = 0

            elif(rospy.get_param('new_ball_detected', 1)):
                stuck_counter = stuck_counter + 1
                if (stuck_counter > STUCK_PATIENCE):
                    rospy.loginfo('Dog: I have spotted a ball through a wall! Well, ' \
                                  'this should not have happened...')
                    rospy.set_param('new_ball_detected', 0)
                    rospy.set_param('stuck', 1)
                    stuck_counter = 0
                    if(blue_solved == 1):
                        blue_solved = 0
                    if(red_solved == 1):
                        red_solved = 0
                    if(green_solved == 1):
                        green_solved = 0
                    if(yellow_solved == 1):
                        yellow_solved = 0
                    if(magenta_solved == 1):
                        magenta_solved = 0
                    if(black_solved == 1):
                        black_solved = 0


        cv2.imshow('Dog camera', image_np)
        cv2.waitKey(2)


## Class used to define terminal colors for the balls
class tcolors:
    RED = '\033[91m'
    GREEN = '\033[92m'
    YELLOW = '\033[93m'
    BLUE = '\033[94m'
    MAGENTA = '\033[95m'
    CYAN = '\033[96m'
    WHITE = '\033[97m'
    ENDC = '\033[0m'


## Invokes the image_feature class and spins until interrupted by a keyboard command
def main(args):
    #Initializes and cleanups ros node
    #ic = image_feature()
    image_feature()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print ("Shutting down ROS Image feature detector module")
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main(sys.argv)
