#!/usr/bin/env python

## @package perception
# Emulates an user's voice command and pointing gestures.
# The user randomly asks the robot to <<play>> and issues a location to reach

import rospy
import time
import random
import assignment2.msg
import actionlib
import actionlib.msg
from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped

## Acquire maximum x-axis parameter from launch file
map_x_max = rospy.get_param('map/x_max')
## Acquire maximum y-axis parameter from launch file
map_y_max = rospy.get_param('map/y_max')
## Acquire minimum x-axis parameter from launch file
map_x_min = rospy.get_param('map/x_min')
## Acquire minimum y-axis parameter from launch file
map_y_min = rospy.get_param('map/y_min')

## Acquire simulation speed scaling factor from launch file
sim_scale = rospy.get_param('sim_scale')

## Simulate the sensors capturing user's <<play>> requests and subsequent
# pointing gestures. The user is assumed to be aware of the robot's current
# state (e.g. via LEDs on the robot itself) and asks to <<play>> only if
# The dog is not sleeping or already playing, i.e. it is in Normal state
def perception():
    rospy.init_node('perception_node', anonymous=True)
    ball_action_client = actionlib.SimpleActionClient('/ball/reaching_goal', assignment2.msg.PlanningAction)
    # oppure 'ball/reaching_goal'
    rate = rospy.Rate(200)
    ball_choice = PoseStamped()
    # raggio sfera 0.5: quando va sotto deve andare a -0.5
    while not rospy.is_shutdown():
        time.sleep(random.randint(6, 7) / sim_scale)
        if rospy.get_param('state') == 'normal':
            human_moves_timer = random.randint(2, 4)
            while (rospy.get_param('state') != 'sleep' and human_moves_timer != 0):
                ball_choice.pose.position.x = random.randint(map_x_min, map_x_max)
                ball_choice.pose.position.y = random.randint(map_y_min, map_y_max)
                hide_variable = random.randint(0, 0) # 0 0 per test, 0 1 default!
                if hide_variable == 0:
                    ball_choice.pose.position.z = 0.5
                    rospy.loginfo('*The human moves the ball to: %i %i*', \
                      ball_choice.pose.position.x, ball_choice.pose.position.y)             
                    ball_goal = assignment2.msg.PlanningGoal(target_pose = ball_choice)
                    ball_action_client.send_goal(ball_goal)
                    ball_action_client.wait_for_result()
                    rospy.loginfo('Ball moved to: %i %i*', \
                      ball_choice.pose.position.x, ball_choice.pose.position.y)
                else:
                    ball_choice.pose.position.z = -0.5
                    rospy.loginfo('*The human hides the ball...*')                  
                    ball_goal = assignment2.msg.PlanningGoal(target_pose = ball_choice)
                    ball_action_client.send_goal(ball_goal)
                    ball_action_client.wait_for_result()
                    rospy.loginfo('Ball hidden!')
                human_moves_timer = human_moves_timer - 1

                time.sleep(random.randint(10, 16) / sim_scale) # qua forse lascio il tempo al robot di girare la testa, o forse no
            rate.sleep()
            ball_choice.pose.position.x = random.randint(map_x_min, map_x_max)
            ball_choice.pose.position.y = random.randint(map_y_min, map_y_max)
            ball_choice.pose.position.z = -0.5
            rospy.loginfo('*The human hides the ball and stops for a while...*')              
            ball_goal = assignment2.msg.PlanningGoal(target_pose = ball_choice)
            ball_action_client.send_goal(ball_goal)
            ball_action_client.wait_for_result()
            rospy.loginfo('Ball hidden for a while!')
    rate.sleep()

if __name__ == '__main__':
    try:
        perception()
    except rospy.ROSInterruptException:
        pass