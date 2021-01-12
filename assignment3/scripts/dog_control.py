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

from assignment3.action import IntAction

## Acquire simulation speed scaling factor from launch file
sim_scale = rospy.get_param('sim_scale')

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
#per decidere basta guardare la prima cifra, a parte tra 0 e 5 in cui
#si guarda la seconda per spareggio nero-rosso. forse non si fa cosi mi sa

blue_solved = 0 # 0 = not yet discovered; 1 = in progress; 2 = done
red_solved = 0
green_solved = 0
yellow_solved = 0
magenta_solved = 0
black_solved = 0

room_color = rospy.get_param('room_color')

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

        #self.ball_pub = rospy.Publisher("ball_detection_topic", Int64, queue_size=1) 
        # potrebbe essere stato utile tenere il vecchio ball. serviva per discernere
        # anche tra play state e normal, mandando 1 o 2 sul topic
        
        # ma non so se serve ora, ho il param new_ball_detected che se funziona e ottimo

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
        #global blueLower, blueUpper, redLower, redUpper, greenLower, greenUpper, \
        #        yellowLower, yellowUpper, magentaLower, magentaUpper, blackLower, blackUpper, \
        #        blue_solved, red_solved, green_solved, yellow_solved, magenta_solved, black_solved
        global blue_solved, red_solved, green_solved, yellow_solved, magenta_solved, black_solved

        if (rospy.get_param('state') == 'normal'):
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
                and red_solved != 1 and green_solved != 1 and yellow_solved != 1 and \
                    magenta_solved != 1 and black_solved != 1):
                    # per il play state (quando cerco la palla posso copiare questo
                    # senza blueCnts, o robe cosi...)
                    #NB ho messo blue solved != 2 perche qui siamo in normal o find, non in play
                rospy.set_param('new_ball_detected', 1)
                blue_solved = 1

                # find the largest contour in the blue mask, then use
                # it to compute the minimum enclosing circle and
                # centroid
                #if not rospy.get_param('state') == 'sleep':
                #    rospy.set_param('ball_detected', 1)

                #self.ball_pub.publish(1)
                c = max(blueCnts, key=cv2.contourArea)
                ((x, y), radius) = cv2.minEnclosingCircle(c)
                M = cv2.moments(c)
                center = (int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"]))

                # only proceed if the radius meets a minimum size
                if(center != 400 or radius != 100):
                    # draw the circle and centroid on the frame,
                    # then update the list of tracked points
                    cv2.circle(image_np, (int(x), int(y)), int(radius), (0, 255, 255), 2) # cerchio giallo
                    cv2.circle(image_np, center, 5, (0, 0, 255), -1) # puntino rosso
                    vel = Twist()
                    vel.angular.z = -0.002 * (center[0] - 400)
                    vel.linear.x = -0.01 * (radius - 100)
                    self.vel_pub.publish(vel)
                else:
                    #qui siamo arrivati nella stanza. segna la posizione
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
                and blue_solved != 1 and green_solved != 1 and yellow_solved != 1 and \
                    magenta_solved != 1 and black_solved != 1):
                    # per il play state (quando cerco la palla posso copiare questo
                    # senza redCnts, o robe cosi...)
                    #NB ho messo red solved != 2 perche qui siamo in normal o find, non in play
                rospy.set_param('new_ball_detected', 1)
                red_solved = 1

                # find the largest contour in the red mask, then use
                # it to compute the minimum enclosing circle and
                # centroid
                #if not rospy.get_param('state') == 'sleep':
                #    rospy.set_param('ball_detected', 1)

                #self.ball_pub.publish(1)
                c = max(redCnts, key=cv2.contourArea)
                ((x, y), radius) = cv2.minEnclosingCircle(c)
                M = cv2.moments(c)
                center = (int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"]))

                # only proceed if the radius meets a minimum size
                if(center != 400 or radius != 100):
                    # draw the circle and centroid on the frame,
                    # then update the list of tracked points
                    cv2.circle(image_np, (int(x), int(y)), int(radius), (0, 255, 255), 2) # cerchio giallo
                    cv2.circle(image_np, center, 5, (0, 255, 0), -1) # puntino verde
                    vel = Twist()
                    vel.angular.z = -0.002 * (center[0] - 400)
                    vel.linear.x = -0.01 * (radius - 100)
                    self.vel_pub.publish(vel)
                else:
                    #qui siamo arrivati nella stanza. segna la posizione
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
                and blue_solved != 1 and red_solved != 1 and yellow_solved != 1 and \
                    magenta_solved != 1 and black_solved != 1):
                    # per il play state (quando cerco la palla posso copiare questo
                    # senza greenCnts, o robe cosi...)
                    #NB ho messo green solved != 2 perche qui siamo in normal o find, non in play
                rospy.set_param('new_ball_detected', 1)
                green_solved = 1

                # find the largest contour in the green mask, then use
                # it to compute the minimum enclosing circle and
                # centroid
                #if not rospy.get_param('state') == 'sleep':
                #    rospy.set_param('ball_detected', 1)

                #self.ball_pub.publish(1)
                c = max(greenCnts, key=cv2.contourArea)
                ((x, y), radius) = cv2.minEnclosingCircle(c)
                M = cv2.moments(c)
                center = (int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"]))

                # only proceed if the radius meets a minimum size
                if(center != 400 or radius != 100):
                    # draw the circle and centroid on the frame,
                    # then update the list of tracked points
                    cv2.circle(image_np, (int(x), int(y)), int(radius), (0, 255, 255), 2) # cerchio giallo
                    cv2.circle(image_np, center, 5, (0, 0, 255), -1) # puntino rosso
                    vel = Twist()
                    vel.angular.z = -0.002 * (center[0] - 400)
                    vel.linear.x = -0.01 * (radius - 100)
                    self.vel_pub.publish(vel)
                else:
                    #qui siamo arrivati nella stanza. segna la posizione
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
                and blue_solved != 1 and red_solved != 1 and green_solved != 1 and \
                    magenta_solved != 1 and black_solved != 1):
                    # per il play state (quando cerco la palla posso copiare questo
                    # senza yellowCnts, o robe cosi...)
                    #NB ho messo yellow solved != 2 perche qui siamo in normal o find, non in play
                rospy.set_param('new_ball_detected', 1)
                yellow_solved = 1

                # find the largest contour in the yellow mask, then use
                # it to compute the minimum enclosing circle and
                # centroid
                #if not rospy.get_param('state') == 'sleep':
                #    rospy.set_param('ball_detected', 1)

                #self.ball_pub.publish(1)
                c = max(yellowCnts, key=cv2.contourArea)
                ((x, y), radius) = cv2.minEnclosingCircle(c)
                M = cv2.moments(c)
                center = (int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"]))

                # only proceed if the radius meets a minimum size
                if(center != 400 or radius != 100):
                    # draw the circle and centroid on the frame,
                    # then update the list of tracked points
                    cv2.circle(image_np, (int(x), int(y)), int(radius), (255, 0, 255), 2) # cerchio magenta
                    cv2.circle(image_np, center, 5, (0, 0, 255), -1) # puntino rosso
                    vel = Twist()
                    vel.angular.z = -0.002 * (center[0] - 400)
                    vel.linear.x = -0.01 * (radius - 100)
                    self.vel_pub.publish(vel)
                else:
                    #qui siamo arrivati nella stanza. segna la posizione
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
                and blue_solved != 1 and red_solved != 1 and green_solved != 1 and \
                    yellow_solved != 1 and black_solved != 1):
                    # per il play state (quando cerco la palla posso copiare questo
                    # senza magentaCnts, o robe cosi...)
                    #NB ho messo magenta solved != 2 perche qui siamo in normal o find, non in play
                rospy.set_param('new_ball_detected', 1)
                magenta_solved = 1

                # find the largest contour in the magenta mask, then use
                # it to compute the minimum enclosing circle and
                # centroid
                #if not rospy.get_param('state') == 'sleep':
                #    rospy.set_param('ball_detected', 1)

                #self.ball_pub.publish(1)
                c = max(magentaCnts, key=cv2.contourArea)
                ((x, y), radius) = cv2.minEnclosingCircle(c)
                M = cv2.moments(c)
                center = (int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"]))

                # only proceed if the radius meets a minimum size
                if(center != 400 or radius != 100):
                    # draw the circle and centroid on the frame,
                    # then update the list of tracked points
                    cv2.circle(image_np, (int(x), int(y)), int(radius), (0, 255, 255), 2) # cerchio giallo
                    cv2.circle(image_np, center, 5, (0, 255, 0), -1) # puntino verde
                    vel = Twist()
                    vel.angular.z = -0.002 * (center[0] - 400)
                    vel.linear.x = -0.01 * (radius - 100)
                    self.vel_pub.publish(vel)
                else:
                    #qui siamo arrivati nella stanza. segna la posizione
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
                and blue_solved != 1 and red_solved != 1 and green_solved != 1 and \
                    yellow_solved != 1 and magenta_solved != 1):
                    # per il play state (quando cerco la palla posso copiare questo
                    # senza blackCnts, o robe cosi...)
                    #NB ho messo black solved != 2 perche qui siamo in normal o find, non in play
                rospy.set_param('new_ball_detected', 1)
                black_solved = 1

                # find the largest contour in the black mask, then use
                # it to compute the minimum enclosing circle and
                # centroid
                #if not rospy.get_param('state') == 'sleep':
                #    rospy.set_param('ball_detected', 1)

                #self.ball_pub.publish(1)
                c = max(blackCnts, key=cv2.contourArea)
                ((x, y), radius) = cv2.minEnclosingCircle(c)
                M = cv2.moments(c)
                center = (int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"]))

                # only proceed if the radius meets a minimum size
                if(center != 400 or radius != 100):
                    # draw the circle and centroid on the frame,
                    # then update the list of tracked points
                    cv2.circle(image_np, (int(x), int(y)), int(radius), (0, 255, 255), 2) # cerchio giallo
                    cv2.circle(image_np, center, 5, (0, 0, 255), -1) # puntino rosso
                    vel = Twist()
                    vel.angular.z = -0.002 * (center[0] - 400)
                    vel.linear.x = -0.01 * (radius - 100)
                    self.vel_pub.publish(vel)
                else:
                    #qui siamo arrivati nella stanza. segna la posizione
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
            # ci vuole il tracking dei colori come per normal
            #ma con la differenza che se vedo il colore obiettivo poi torno in play
            #devo capire quale palla cercare e vedere se trovo altre nuove

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
                and red_solved != 1 and green_solved != 1 and yellow_solved != 1 and \
                    magenta_solved != 1 and black_solved != 1):
                    # per il play state (quando cerco la palla posso copiare questo
                    # senza blueCnts, o robe cosi...)
                    #NB ho messo blue solved != 2 perche qui siamo in normal o find, non in play
                rospy.set_param('new_ball_detected', 1)
                blue_solved = 1
                explore_client = actionlib.SimpleActionClient('explore', IntAction)
                #explore_client.send_goal(0)
                explore_client.cancel_all_goals()

                # find the largest contour in the blue mask, then use
                # it to compute the minimum enclosing circle and
                # centroid
                #if not rospy.get_param('state') == 'sleep':
                #    rospy.set_param('ball_detected', 1)

                #self.ball_pub.publish(1)
                c = max(blueCnts, key=cv2.contourArea)
                ((x, y), radius) = cv2.minEnclosingCircle(c)
                M = cv2.moments(c)
                center = (int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"]))

                # only proceed if the radius meets a minimum size
                if(center != 400 or radius != 100):
                    # draw the circle and centroid on the frame,
                    # then update the list of tracked points
                    cv2.circle(image_np, (int(x), int(y)), int(radius), (0, 255, 255), 2) # cerchio giallo
                    cv2.circle(image_np, center, 5, (0, 0, 255), -1) # puntino rosso
                    vel = Twist()
                    vel.angular.z = -0.002 * (center[0] - 400)
                    vel.linear.x = -0.01 * (radius - 100)
                    self.vel_pub.publish(vel)
                else:
                    #qui siamo arrivati nella stanza. segna la posizione
                    pos = rospy.wait_for_message('odom', Odometry, timeout = None)
                    #pos = Odometry()
                    rospy.set_param('blue/x', pos.pose.pose.position.x)
                    rospy.set_param('blue/y', pos.pose.pose.position.y)
                    rospy.set_param('dog/x', pos.pose.pose.position.x)
                    rospy.set_param('dog/y', pos.pose.pose.position.y)
                    blue_solved = 2
                    rospy.set_param('new_ball_detected', 0)
                    if rospy.get_param('unknown_ball') == 0:
                        rospy.set_param('unknown_ball', 100)
                    else:
                        explore_client.send_goal(1)

                image_np = cv2.rotate(image_np, cv2.ROTATE_90_CLOCKWISE)
                cv2.imshow('window', image_np)
                cv2.waitKey(2)

            elif(len(redCnts) > 0 and red_solved != 2 \
                and blue_solved != 1 and green_solved != 1 and yellow_solved != 1 and \
                    magenta_solved != 1 and black_solved != 1):
                    # per il play state (quando cerco la palla posso copiare questo
                    # senza redCnts, o robe cosi...)
                    #NB ho messo red solved != 2 perche qui siamo in normal o find, non in play
                rospy.set_param('new_ball_detected', 1)
                red_solved = 1
                explore_client = actionlib.SimpleActionClient('explore', IntAction)
                #explore_client.send_goal(0)
                explore_client.cancel_all_goals()

                # find the largest contour in the red mask, then use
                # it to compute the minimum enclosing circle and
                # centroid
                #if not rospy.get_param('state') == 'sleep':
                #    rospy.set_param('ball_detected', 1)

                #self.ball_pub.publish(1)
                c = max(redCnts, key=cv2.contourArea)
                ((x, y), radius) = cv2.minEnclosingCircle(c)
                M = cv2.moments(c)
                center = (int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"]))

                # only proceed if the radius meets a minimum size
                if(center != 400 or radius != 100):
                    # draw the circle and centroid on the frame,
                    # then update the list of tracked points
                    cv2.circle(image_np, (int(x), int(y)), int(radius), (0, 255, 255), 2) # cerchio giallo
                    cv2.circle(image_np, center, 5, (0, 255, 0), -1) # puntino verde
                    vel = Twist()
                    vel.angular.z = -0.002 * (center[0] - 400)
                    vel.linear.x = -0.01 * (radius - 100)
                    self.vel_pub.publish(vel)
                else:
                    #qui siamo arrivati nella stanza. segna la posizione
                    pos = rospy.wait_for_message('odom', Odometry, timeout = None)
                    #pos = Odometry()
                    rospy.set_param('red/x', pos.pose.pose.position.x)
                    rospy.set_param('red/y', pos.pose.pose.position.y)
                    rospy.set_param('dog/x', pos.pose.pose.position.x)
                    rospy.set_param('dog/y', pos.pose.pose.position.y)
                    red_solved = 2
                    rospy.set_param('new_ball_detected', 0)
                    if rospy.get_param('unknown_ball') == 1:
                        rospy.set_param('unknown_ball', 100)
                    else:
                        explore_client.send_goal(1)
                       
                image_np = cv2.rotate(image_np, cv2.ROTATE_90_CLOCKWISE)
                cv2.imshow('window', image_np)
                cv2.waitKey(2)

            elif(len(greenCnts) > 0 and green_solved != 2 \
                and blue_solved != 1 and red_solved != 1 and yellow_solved != 1 and \
                    magenta_solved != 1 and black_solved != 1):
                    # per il play state (quando cerco la palla posso copiare questo
                    # senza greenCnts, o robe cosi...)
                    #NB ho messo green solved != 2 perche qui siamo in normal o find, non in play
                rospy.set_param('new_ball_detected', 1)
                green_solved = 1
                explore_client = actionlib.SimpleActionClient('explore', IntAction)
                #explore_client.send_goal(0)
                explore_client.cancel_all_goals()

                # find the largest contour in the green mask, then use
                # it to compute the minimum enclosing circle and
                # centroid
                #if not rospy.get_param('state') == 'sleep':
                #    rospy.set_param('ball_detected', 1)

                #self.ball_pub.publish(1)
                c = max(greenCnts, key=cv2.contourArea)
                ((x, y), radius) = cv2.minEnclosingCircle(c)
                M = cv2.moments(c)
                center = (int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"]))

                # only proceed if the radius meets a minimum size
                if(center != 400 or radius != 100):
                    # draw the circle and centroid on the frame,
                    # then update the list of tracked points
                    cv2.circle(image_np, (int(x), int(y)), int(radius), (0, 255, 255), 2) # cerchio giallo
                    cv2.circle(image_np, center, 5, (0, 0, 255), -1) # puntino rosso
                    vel = Twist()
                    vel.angular.z = -0.002 * (center[0] - 400)
                    vel.linear.x = -0.01 * (radius - 100)
                    self.vel_pub.publish(vel)
                else:
                    #qui siamo arrivati nella stanza. segna la posizione
                    pos = rospy.wait_for_message('odom', Odometry, timeout = None)
                    #pos = Odometry()
                    rospy.set_param('green/x', pos.pose.pose.position.x)
                    rospy.set_param('green/y', pos.pose.pose.position.y)
                    rospy.set_param('dog/x', pos.pose.pose.position.x)
                    rospy.set_param('dog/y', pos.pose.pose.position.y)
                    green_solved = 2
                    rospy.set_param('new_ball_detected', 0)
                    if rospy.get_param('unknown_ball') == 2:
                        rospy.set_param('unknown_ball', 100)
                    else:
                        explore_client.send_goal(1)
 
                image_np = cv2.rotate(image_np, cv2.ROTATE_90_CLOCKWISE)
                cv2.imshow('window', image_np)
                cv2.waitKey(2)

            elif(len(yellowCnts) > 0 and yellow_solved != 2 \
                and blue_solved != 1 and red_solved != 1 and green_solved != 1 and \
                    magenta_solved != 1 and black_solved != 1):
                    # per il play state (quando cerco la palla posso copiare questo
                    # senza yellowCnts, o robe cosi...)
                    #NB ho messo yellow solved != 2 perche qui siamo in normal o find, non in play
                rospy.set_param('new_ball_detected', 1)
                yellow_solved = 1
                explore_client = actionlib.SimpleActionClient('explore', IntAction)
                #explore_client.send_goal(0)
                explore_client.cancel_all_goals()

                # find the largest contour in the yellow mask, then use
                # it to compute the minimum enclosing circle and
                # centroid
                #if not rospy.get_param('state') == 'sleep':
                #    rospy.set_param('ball_detected', 1)

                #self.ball_pub.publish(1)
                c = max(yellowCnts, key=cv2.contourArea)
                ((x, y), radius) = cv2.minEnclosingCircle(c)
                M = cv2.moments(c)
                center = (int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"]))

                # only proceed if the radius meets a minimum size
                if(center != 400 or radius != 100):
                    # draw the circle and centroid on the frame,
                    # then update the list of tracked points
                    cv2.circle(image_np, (int(x), int(y)), int(radius), (255, 0, 255), 2) # cerchio magenta
                    cv2.circle(image_np, center, 5, (0, 0, 255), -1) # puntino rosso
                    vel = Twist()
                    vel.angular.z = -0.002 * (center[0] - 400)
                    vel.linear.x = -0.01 * (radius - 100)
                    self.vel_pub.publish(vel)
                else:
                    #qui siamo arrivati nella stanza. segna la posizione
                    pos = rospy.wait_for_message('odom', Odometry, timeout = None)
                    #pos = Odometry()
                    rospy.set_param('yellow/x', pos.pose.pose.position.x)
                    rospy.set_param('yellow/y', pos.pose.pose.position.y)
                    rospy.set_param('dog/x', pos.pose.pose.position.x)
                    rospy.set_param('dog/y', pos.pose.pose.position.y)
                    yellow_solved = 2
                    rospy.set_param('new_ball_detected', 0)
                    if rospy.get_param('unknown_ball') == 3:
                        rospy.set_param('unknown_ball', 100)
                    else:
                        explore_client.send_goal(1)
 
                image_np = cv2.rotate(image_np, cv2.ROTATE_90_CLOCKWISE)
                cv2.imshow('window', image_np)
                cv2.waitKey(2)

            elif(len(magentaCnts) > 0 and magenta_solved != 2 \
                and blue_solved != 1 and red_solved != 1 and green_solved != 1 and \
                    yellow_solved != 1 and black_solved != 1):
                    # per il play state (quando cerco la palla posso copiare questo
                    # senza magentaCnts, o robe cosi...)
                    #NB ho messo magenta solved != 2 perche qui siamo in normal o find, non in play
                rospy.set_param('new_ball_detected', 1)
                magenta_solved = 1
                explore_client = actionlib.SimpleActionClient('explore', IntAction)
                #explore_client.send_goal(0)
                explore_client.cancel_all_goals()

                # find the largest contour in the magenta mask, then use
                # it to compute the minimum enclosing circle and
                # centroid
                #if not rospy.get_param('state') == 'sleep':
                #    rospy.set_param('ball_detected', 1)

                #self.ball_pub.publish(1)
                c = max(magentaCnts, key=cv2.contourArea)
                ((x, y), radius) = cv2.minEnclosingCircle(c)
                M = cv2.moments(c)
                center = (int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"]))

                # only proceed if the radius meets a minimum size
                if(center != 400 or radius != 100):
                    # draw the circle and centroid on the frame,
                    # then update the list of tracked points
                    cv2.circle(image_np, (int(x), int(y)), int(radius), (0, 255, 255), 2) # cerchio giallo
                    cv2.circle(image_np, center, 5, (0, 255, 0), -1) # puntino verde
                    vel = Twist()
                    vel.angular.z = -0.002 * (center[0] - 400)
                    vel.linear.x = -0.01 * (radius - 100)
                    self.vel_pub.publish(vel)
                else:
                    #qui siamo arrivati nella stanza. segna la posizione
                    pos = rospy.wait_for_message('odom', Odometry, timeout = None)
                    #pos = Odometry()
                    rospy.set_param('magenta/x', pos.pose.pose.position.x)
                    rospy.set_param('magenta/y', pos.pose.pose.position.y)
                    rospy.set_param('dog/x', pos.pose.pose.position.x)
                    rospy.set_param('dog/y', pos.pose.pose.position.y)
                    magenta_solved = 2
                    rospy.set_param('new_ball_detected', 0)
                    if rospy.get_param('unknown_ball') == 4:
                        rospy.set_param('unknown_ball', 100)
                    else:
                        explore_client.send_goal(1)
 
                image_np = cv2.rotate(image_np, cv2.ROTATE_90_CLOCKWISE)
                cv2.imshow('window', image_np)
                cv2.waitKey(2)

            elif(len(blackCnts) > 0 and black_solved != 2 \
                and blue_solved != 1 and red_solved != 1 and green_solved != 1 and \
                    yellow_solved != 1 and magenta_solved != 1):
                    # per il play state (quando cerco la palla posso copiare questo
                    # senza blackCnts, o robe cosi...)
                    #NB ho messo black solved != 2 perche qui siamo in normal o find, non in play
                rospy.set_param('new_ball_detected', 1)
                black_solved = 1
                explore_client = actionlib.SimpleActionClient('explore', IntAction)
                #explore_client.send_goal(0)
                explore_client.cancel_all_goals()

                # find the largest contour in the black mask, then use
                # it to compute the minimum enclosing circle and
                # centroid
                #if not rospy.get_param('state') == 'sleep':
                #    rospy.set_param('ball_detected', 1)

                #self.ball_pub.publish(1)
                c = max(blackCnts, key=cv2.contourArea)
                ((x, y), radius) = cv2.minEnclosingCircle(c)
                M = cv2.moments(c)
                center = (int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"]))

                # only proceed if the radius meets a minimum size
                if(center != 400 or radius != 100):
                    # draw the circle and centroid on the frame,
                    # then update the list of tracked points
                    cv2.circle(image_np, (int(x), int(y)), int(radius), (0, 255, 255), 2) # cerchio giallo
                    cv2.circle(image_np, center, 5, (0, 0, 255), -1) # puntino rosso
                    vel = Twist()
                    vel.angular.z = -0.002 * (center[0] - 400)
                    vel.linear.x = -0.01 * (radius - 100)
                    self.vel_pub.publish(vel)
                else:
                    #qui siamo arrivati nella stanza. segna la posizione
                    pos = rospy.wait_for_message('odom', Odometry, timeout = None)
                    #pos = Odometry()
                    rospy.set_param('black/x', pos.pose.pose.position.x)
                    rospy.set_param('black/y', pos.pose.pose.position.y)
                    rospy.set_param('dog/x', pos.pose.pose.position.x)
                    rospy.set_param('dog/y', pos.pose.pose.position.y)
                    black_solved = 2
                    rospy.set_param('new_ball_detected', 0)
                    if rospy.get_param('unknown_ball') == 5:
                        rospy.set_param('unknown_ball', 100)
                    else:
                        explore_client.send_goal(1)
 
                image_np = cv2.rotate(image_np, cv2.ROTATE_90_CLOCKWISE)
                cv2.imshow('window', image_np)
                cv2.waitKey(2)


            #qui ce un action client che comunica 0 a explore.cpp quando abbiamo fatto
            rospy.set_param('find_task_status', 0)



        elif(rospy.get_param('state') == 'play' or rospy.get_param('state') == 'sleep'): # basta anche solo un else qui
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
    ic = image_feature()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print ("Shutting down ROS Image feature detector module")
    cv2.destroyAllWindows()


if __name__ == '__main__':
    main(sys.argv)
