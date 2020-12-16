#!/usr/bin/env python

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

from std_msgs.msg import Int64

find_counter = 0

class image_feature:

    def __init__(self):
        '''Initialize ros publisher, ros subscriber'''
        rospy.init_node('image_feature', anonymous=True)
     # topic where we publish
        self.image_pub = rospy.Publisher("output/image_raw/compressed",
                                         CompressedImage, queue_size=1)
        self.vel_pub = rospy.Publisher("cmd_vel",
                                       Twist, queue_size=1)

        self.ball_pub = rospy.Publisher("ball_control_topic", Int64, queue_size=1) 

        # subscribed Topic
        self.subscriber = rospy.Subscriber("camera1/image_raw/compressed",
                                           CompressedImage, self.callback,  queue_size=1)

    def callback(self, ros_data):
        '''Callback function of subscribed topic. 
        Here images get converted and features detected'''
        global find_counter

        if (rospy.get_param('state') == 'normal' or rospy.get_param('state') == 'play'):
            #### direct conversion to CV2 ####
            np_arr = np.fromstring(ros_data.data, np.uint8)
            image_np = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)  # OpenCV >= 3.0:

            orangeLower = (10, 50, 20)
            orangeUpper = (20, 255, 255)

            blurred = cv2.GaussianBlur(image_np, (11, 11), 0)
            hsv = cv2.cvtColor(blurred, cv2.COLOR_BGR2HSV)
            mask = cv2.inRange(hsv, orangeLower, orangeUpper)
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
                find_counter = 0
                c = max(cnts, key=cv2.contourArea)
                ((x, y), radius) = cv2.minEnclosingCircle(c)
                M = cv2.moments(c)
                center = (int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"]))

                self.ball_pub.publish(1)

                # only proceed if the radius meets a minimum size
                if radius > 10:
                    # draw the circle and centroid on the frame,
                    # then update the list of tracked points
                    cv2.circle(image_np, (int(x), int(y)), int(radius),
                            (0, 255, 255), 2)
                    cv2.circle(image_np, center, 5, (0, 0, 255), -1)
                    if rospy.get_param('state') == 'play':
                        vel = Twist()
                        vel.angular.z = -0.002*(center[0]-400)
                        vel.linear.x = -0.01*(radius-100)
                        self.vel_pub.publish(vel)
                elif (radius < 10 and rospy.get_param('state') == 'play'):
                    vel = Twist()
                    vel.linear.x = 0.5
                    self.vel_pub.publish(vel)
                else:
                    print('qui devo mettere che gira la testa!')
                    # poi devo metterlo come if head_moved == 0 printa quello e metti
                    # head_move = 1. negli altri due casi sopra metti come primo
                    # instruction head_moved = 0. e devo metterlo anche a inizio i len(cnts)
                    # per non avere problemi. su questa ultiam cosa bisogna vedere dove si
                    # potrebbe mettere (perche forse non e il posto migliore), ma dovrebbe
                    # andare cosi

            elif (rospy.get_param('state') == 'play' and find_counter < 14): # per niente sicuro di questo
                vel = Twist()
                vel.angular.z = 0.5
                self.vel_pub.publish(vel)
                find_counter = find_counter + 1
            elif (rospy.get_param('state') == 'play' and find_counter >= 14):
                self.ball_pub.publish(2)           

            cv2.imshow('window', image_np)
            cv2.waitKey(2)

            # self.subscriber.unregister()


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
