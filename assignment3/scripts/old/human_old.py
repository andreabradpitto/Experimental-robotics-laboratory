#!/usr/bin/env python

## @package human
# Emulates a human agent which throws or hides a ball.
# The user randomly chooses what to do with the ball;
# when they have chosen, is sends a goal action to go_to_point_ball.py

import rospy
import time
import random
import math
import assignment3.msg
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
home_x = rospy.get_param('home/x')
home_y = rospy.get_param('home/y')
room_list = rospy.get_param('room_list')
#room_list = ['entrance', 'closet', 'livingroom', 'kitchen', 'bathroom', 'bedroom']

## Acquire simulation speed scaling factor from launch file
sim_scale = rospy.get_param('sim_scale')

## After a pause, simulate the human behavior by randomly picking the next
# thing to do. The human can either hide or move the ball over the playing field
def human():
    rospy.init_node('human_node', anonymous=True)
    room_action_client = actionlib.SimpleActionClient('/reaching_room_topic', assignment3.msg.roomPlanningAction)
    home_action_client = actionlib.SimpleActionClient('/reaching_home_topic', assignment3.msg.homePlanningAction)
    # puo essere da sistemare topics sopra: /reaching_goal_topic
    rate = rospy.Rate(200)
    room_name = String()
    home_pos = PoseStamped()
    home_pos.pose.position.x = home_x
    home_pos.pose.position.y = home_y
    voice_pub = rospy.Publisher('play_topic', String, queue_size=10)
    while not rospy.is_shutdown():
        time.sleep(random.randint(30, 70) / sim_scale)
        if rospy.get_param('state') == 'normal':
            voice_pub.publish('play')
            rospy.loginfo('Human: I want to play')
            while (rospy.get_param('state') == 'normal'):
                rate.sleep()
            while (rospy.get_param('state') == 'play'):
                room_choice = random.randint(0, 5)
                string = 'Human: GoTo ' + room_list[room_choice]
                rospy.loginfo(string)
                room_name = room_list[room_choice]
                room_goal = assignment3.msg.roomPlanningGoal(target_pose = room_name)
                room_action_client.send_goal(room_goal)
                room_action_client.wait_for_result()
                rospy.loginfo('Human: Ok, now come back here!') #da eliminare         
                home_goal = assignment3.msg.homePlanningGoal(target_pose = home_pos)
                home_action_client.send_goal(home_goal)
                home_action_client.wait_for_result()
                rospy.loginfo('Human: home reached') #da eliminare
                #con questa scelta nella fsm devo mettere il stop play counter (go back to normal)
                #che si basa su if last position requested (o controlla lui la sua posizione attuale)
                # == home allora puoi smettere. non voglio che smetta mentre è a metà di sto coso
                #altrimenti si blocca tutto
                time.sleep(random.randint(5, 10) / sim_scale)
            rate.sleep()
            rospy.loginfo('Human: The robot has stopped playing') #da eliminare
            time.sleep(random.randint(50, 100) / sim_scale)#da aumentare
    rate.sleep()

if __name__ == '__main__':
    try:
        human()
    except rospy.ROSInterruptException:
        pass