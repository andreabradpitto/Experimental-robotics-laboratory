#!/usr/bin/env python

## @package dog_fsm
# Emulates the robotic dog's finite state machine internal architecture. 
# The implemented states are Sleep, Normal, Play

import roslib
import rospy
import smach
import smach_ros
import time
import random
import assignment2
from std_msgs.msg import String, Int64
from assignment2.msg import Coordinates

## Acquire maximum x-axis parameter from launch file
map_x_max = rospy.get_param('map/x_max')
## Acquire maximum y-axis parameter from launch file
map_y_max = rospy.get_param('map/y_max')
## Acquire minimum x-axis parameter from launch file
map_x_min = rospy.get_param('map/x_min')
## Acquire minimum y-axis parameter from launch file
map_y_min = rospy.get_param('map/y_min')

## Acquire x-axis home position parameter from launch file
home_x = rospy.get_param('home/x')
## Acquire y-axis home position parameter from launch file
home_y = rospy.get_param('home/y')

## Acquire simulation speed scaling factor from launch file
sim_scale = rospy.get_param('sim_scale')

## variable used to let the fsm behave differently for the very first state only
first_iteration = 1

## variable used to state whether it is time to play or not
playtime = 0

## Sleep state definition
class Sleep(smach.State):
    ## Sleep state initialization: set the outcomes and subscribe to the
    # motion_over_topic topic
    def __init__(self):
        # initialisation function, it should not wait
        smach.State.__init__(self, 
                             outcomes=['wake_up'])
        rospy.Subscriber('motion_over_topic', Coordinates, self.sleep_callback)

    ## Sleep state execution: the robot goes to sleep once he gets back home
    def execute(self, userdata):
        # function called when exiting from the node, it can be blocking
        global first_iteration
        rospy.set_param('state','sleep')
        pos = Coordinates()
        pos.x = home_x # reach home position (x-axis)
        pos.y = home_y # reach home position (y-axis)
        pub_sleep = rospy.Publisher('control_topic', Coordinates, queue_size=10)
        if first_iteration == 0:
            rospy.loginfo('dog: I am going to spleep!')
            time.sleep(random.randint(2, 5) / sim_scale)
        else:
            time.sleep(random.randint(2, 5) / sim_scale)
        timer = 1
        while (timer != 0 and (not rospy.is_shutdown()) and \
            rospy.get_param('state') == 'sleep'):   
            pub_sleep.publish(pos)
            if(rospy.wait_for_message('motion_over_topic', Coordinates)):
                timer = 0
        time.sleep(random.randint(2, 5) / sim_scale) # the dog is sleeping
        rospy.loginfo('dog: Good morning!')
        return 'wake_up'

    ## Sleep state callback: prints a string once home position has been reached
    # and sets home as the current robot position
    def sleep_callback(self, data):
        global home_x
        global home_y
        global first_iteration
        if (rospy.get_param('state') == 'sleep' and first_iteration == 0):
            rospy.loginfo('dog: home position reached!')
            rospy.set_param('dog/x', home_x)
            rospy.set_param('dog/y', home_y)
        elif(rospy.get_param('state') == 'sleep'):
            rospy.set_param('dog/x', home_x)
            rospy.set_param('dog/y', home_y)
            first_iteration = 0

## Normal state definition
class Normal(smach.State):
    ## Normal state initialization: set the outcomes and subscribe to the
    # play_topic and the motion_over_topic topics
    def __init__(self):
        # initialisation function, it should not wait
        smach.State.__init__(self, 
                             outcomes=['go_play','go_sleep'])
        rospy.Subscriber('motion_over_topic', Coordinates, self.normal_callback_motion)
        rospy.Subscriber('ball_control_topic', Int64, self.normal_callback_ball)
    
    ## Normal state execution: the robot wanders randomly, while waiting for
    # finding the ball, or becoming tired and then going to sleep
    def execute(self, userdata):
        # function called when exiting from the node, it can be blocking
        global playtime
        sleep_timer = (random.randint(2, 7) / sim_scale)
        self.rate = rospy.Rate(200)
        rospy.set_param('state', 'normal')
        pos = Coordinates()
        pub = rospy.Publisher('control_topic', Coordinates, queue_size=10)
        while (sleep_timer != 0 and (not rospy.is_shutdown()) and \
            rospy.get_param('state') == 'normal' and playtime == 0):
            sleep_timer = sleep_timer - 1
            pos.x = random.randint(map_x_min, map_x_max)
            pos.y = random.randint(map_y_min, map_y_max)
            rospy.loginfo('dog: I am moving to %i %i', pos.x, pos.y)	
            pub.publish(pos)
            if(rospy.wait_for_message('motion_over_topic', Coordinates) or playtime == 1):
                rospy.set_param('dog/x', pos.x)
                rospy.set_param('dog/y', pos.y)
            self.rate.sleep

        if sleep_timer == 0:
            return 'go_sleep'

        elif playtime == 1:
            return 'go_play'

    ## Normal state callback that prints a string once the random target
    # position has been reached
    def normal_callback_motion(self, data):
        if (rospy.get_param('state') == 'normal' and rospy.get_param('ball_detected') == 0):
            rospy.loginfo('dog: %i %i position reached!', data.x, data.y)

    ## Normal state callback that prints a string once the robotic dog
    # acknowledges the ball
    def normal_callback_ball(self, data):
        global playtime
        if (rospy.get_param('state') == 'normal' and rospy.get_param('ball_detected') == 1):
            rospy.loginfo('dog: I have seen the ball! Woof!')
            playtime = 1

## Play state definition
class Play(smach.State):
    ## Play state initialization: set the outcomes and subscribe to the
    # ball_detection and the motion_over_topic topics
    def __init__(self):
        # initialisation function, it should not wait
        smach.State.__init__(self, 
                             outcomes=['game_over'])
        rospy.Subscriber('ball_control_topic', Int64, self.play_callback_ball)

    ## Play state execution: the robotic dog follows the ball as long as
    # it is in his sight. If the ball is still, the dog starts turning
    # it head. If the ball is lost over a certain amount of time, the 
    # robot gets back to the Normal state
    def execute(self, userdata):
        # function called when exiting from the node, it can be blocking
        self.rate = rospy.Rate(200)
        rospy.set_param('state', 'play')
        while ((not rospy.is_shutdown()) and playtime == 1 \
            and rospy.get_param('state') == 'play'):
            self.rate.sleep
        return 'game_over'

    ## Play state callback: as the dog has lost the ball, order the
    # finite state machine to get back to the Normal state
    def play_callback_ball(self, data):
        global playtime
        if (rospy.get_param('state') == 'play' and rospy.get_param('ball_detected') == 0):
            rospy.loginfo('dog: I have lost the ball :(')
            playtime = 0

## Finite state machine's (fsm) main. It initializes the dog_fsm_node and setups
# a SMACH state machine along with all the three possible states
def main():
    rospy.init_node('dog_fsm_node')

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
                               transitions={'game_over':'NORMAL'})

    # Create and start the introspection server for visualization
    sis = smach_ros.IntrospectionServer('server_name', sm, '/SM_ROOT')
    sis.start()

    # Execute the state machine
    # outcome = sm.execute() # an output variable is to be used if
                             # this finite state machine is nested
                             # inside another one
    sm.execute()

    # Wait for ctrl-c to stop the application
    rospy.spin()
    sis.stop()

if __name__ == '__main__':
    main()