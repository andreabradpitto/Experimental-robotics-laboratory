#!/usr/bin/env python

## @package command_manager
# Emulates the robot's finite state machine internal architecture
# The implemented states are Sleep, Normal, Play.

import roslib
import rospy
import smach
import smach_ros
import time
import random
from std_msgs.msg import String
from assignment1.msg import Coordinates

## Acquire map size parameters from launch file
map_x_max = rospy.get_param('map/x_max')
map_y_max = rospy.get_param('map/y_max')

## Acquire home position parameters from launch file
home_x = rospy.get_param('home/x')
home_y = rospy.get_param('home/y')

## Acquire person's position parameters from launch file
person_x = rospy.get_param('person/x')
person_y = rospy.get_param('person/y')

## parameter used to let the fsm behave differently for the very first state only
first_iteration = 1

# define state Sleep
class Sleep(smach.State):
    def __init__(self):
        # initialisation function, it should not wait
        smach.State.__init__(self, 
                             outcomes=['wake_up'])
        rospy.Subscriber('motion_over_topic', Coordinates, self.sleep_callback)

    def execute(self, userdata):
        # function called when exiting from the node, it can be blocking
        global first_iteration
        rospy.set_param('state', 'sleep')
        pos = Coordinates()
        pos.x = home_x # reach home position
        pos.y = home_y # reach home position
        if first_iteration == 0:
            rospy.loginfo('MiRo: I am going to spleep!')
        first_iteration = 0
        pub = rospy.Publisher('control_topic', Coordinates, queue_size=10)
        pub.publish(pos)
        if first_iteration == 1:
            rospy.wait_for_message('motion_over_topic', Coordinates)
        time.sleep(random.randint(2, 5)) # MiRo is sleeping
        rospy.loginfo('MiRo: Good morning!')
        return 'wake_up'

    def sleep_callback(self, data):
        global home_x
        global home_y
        if rospy.get_param('state') == 'sleep':
            rospy.loginfo('MiRo: home position reached!')
            rospy.set_param('MiRo/x', home_x)
            rospy.set_param('MiRo/y', home_y)


# define state Normal
class Normal(smach.State):
    def __init__(self):
        # initialisation function, it should not wait
        smach.State.__init__(self, 
                             outcomes=['go_play','go_sleep'])
        rospy.Subscriber('motion_over_topic', Coordinates, self.normal_callback_motion)
        rospy.Subscriber('play_topic', String, self.normal_callback_play)
        
    def execute(self, userdata):
        # function called when exiting from the node, it can be blocking
        sleep_timer = random.randint(2, 10)
        self.rate = rospy.Rate(200)
        rospy.set_param('state', 'normal')
        pos = Coordinates()
        while (sleep_timer != 0 and not rospy.is_shutdown() and rospy.get_param('state') == 'normal'):
            sleep_timer = sleep_timer - 1
            pos.x = random.randint(0, map_x_max)
            pos.y = random.randint(0, map_y_max)
            rospy.loginfo('MiRo: I am moving to %i %i', pos.x, pos.y)	
            pub = rospy.Publisher('control_topic', Coordinates, queue_size=10)
            pub.publish(pos)
            rospy.wait_for_message('motion_over_topic', Coordinates)
            rospy.set_param('MiRo/x', pos.x) # not ideal (unlike in sleep)
            rospy.set_param('MiRo/y', pos.y)
            time.sleep(2)
            if sleep_timer == 0:
                return 'go_sleep'
            self.rate.sleep

    def normal_callback_motion(self, data):
        if rospy.get_param('state') == 'normal':
            rospy.loginfo('MiRo: target position reached!')

    def normal_callback_play(self, data):
        if rospy.get_param('state') == 'normal':        
            rospy.loginfo('MiRo: Ok, let\'s play!')
            return 'go_play'


# define state Play
class Play(smach.State):
    def __init__(self):
        # initialisation function, it should not wait
        smach.State.__init__(self, 
                             outcomes=['go_sleep','game_over'])
        rospy.Subscriber('motion_over_topic', Coordinates, self.play_callback_motion)
        rospy.Subscriber('gesture_topic', Coordinates, self.play_callback_gesture)
        
    def execute(self, userdata):
        # function called when exiting from the node, it can be blocking
        # reach user position
        # ask, then reach pointed or told position
        # do it for some time
        # in the end go to normal,but also here we need sleep timer too
        normal_timer = random.randint(4, 10)
        self.rate = rospy.Rate(200)
        rospy.set_param('state', 'play')
        pos = Coordinates()
        while (normal_timer != 0 and not rospy.is_shutdown() and rospy.get_param('state') == 'play'):
            normal_timer = normal_timer - 1
            rospy.loginfo('MiRo: I am coming to you!')
            pos.x = rospy.get_param('person/x')
            pos.y = rospy.get_param('person/y')
            pub = rospy.Publisher('control_topic', Coordinates, queue_size=10)
            pub.publish(pos)
            rospy.wait_for_message('motion_over_topic', Coordinates) # mi sa che devo cambiare topic per ogni state
            time.sleep(2)
            if normal_timer == 0:
                return 'go_sleep'
            self.rate.sleep

    def play_callback_motion(self, data):
        if rospy.get_param('state') == 'play':
            rospy.loginfo('MiRo: target position reached!')

    def play_callback_gesture(self, data):
        if rospy.get_param('state') == 'play':
            rospy.loginfo('MiRo: I am moving to getured %i %i', data.x, data.y)
            pub = rospy.Publisher('control_topic', Coordinates, queue_size=10)
            pub.publish(data)
            rospy.wait_for_message('motion_over_topic', Coordinates)

        
def main():
    rospy.init_node('miro_fsm_node')

    # Create a SMACH state machine
    sm = smach.StateMachine(outcomes=['container_interface'])
    # Open the container
    with sm:
        # Add states to the container
        smach.StateMachine.add('SLEEP', Sleep(), 
                               transitions={'wake_up':'NORMAL'})
        smach.StateMachine.add('NORMAL', Normal(), 
                               transitions={'go_play':'PLAY', 
                                            'go_sleep':'SLEEP'})
        smach.StateMachine.add('PLAY', Play(), 
                               transitions={'go_sleep':'SLEEP', 
                                            'game_over':'NORMAL'})


    # Create and start the introspection server for visualization
    sis = smach_ros.IntrospectionServer('server_name', sm, '/SM_ROOT')
    sis.start()

    # Execute the state machine
    outcome = sm.execute()

    # Wait for ctrl-c to stop the application
    rospy.spin()
    sis.stop()


if __name__ == '__main__':
    main()