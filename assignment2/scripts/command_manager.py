#!/usr/bin/env python

## @package command_manager
# Emulates the robot's finite state machine internal architecture. 
# The implemented states are Sleep, Normal, Play

import roslib
import rospy
import smach
import smach_ros
import time
import random
import assignment2
from std_msgs.msg import String
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

## Acquire person's x-axis position parameter from launch file
person_x = rospy.get_param('person/x')
## Acquire person's y-axis position parameter from launch file
person_y = rospy.get_param('person/y')

## Acquire simulation speed scaling factor from launch file
sim_scale = rospy.get_param('sim_scale')

## variable used to let the fsm behave differently for the very first state only
first_iteration = 1

## variable used in Normal state to flag when it is time to play
playtime = 0

## vairable used to store the gestured point coordinates
gestured_point = Coordinates()
gestured_point.x = 0
gestured_point.y = 0

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
            time.sleep(2)
        else:
            time.sleep(2)
        timer = 1
        while (timer != 0 and not rospy.is_shutdown() and \
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
        #rospy.Subscriber('play_topic', String, self.normal_callback_play)

        # qua devo mettere un suscriber per quando viene vista la palla
        # in quel caso metto playtime = 1 nella callback e cosi
        # vai in play state. devo capire chi e che controlla e come
        # quando sta palal viene vista
        #per forza qualche controllo nelnormal state...
    
    ## Normal state execution: the robot wanders randomly, while waiting for
    # user <<play>> requests and for going to sleep
    def execute(self, userdata):
        # function called when exiting from the node, it can be blocking
        global playtime
        playtime = 0
        sleep_timer = random.randint(2, 7)
        self.rate = rospy.Rate(200)
        rospy.set_param('state', 'normal')
        pos = Coordinates()
        pub = rospy.Publisher('control_topic', Coordinates, queue_size=10)
        while (sleep_timer != 0 and not rospy.is_shutdown() and \
            rospy.get_param('state') == 'normal' and playtime == 0):
            sleep_timer = sleep_timer - 1
            pos.x = random.randint(map_x_min, map_x_max)
            pos.y = random.randint(map_y_min, map_y_max)
            rospy.loginfo('dog: I am moving to %i %i', pos.x, pos.y)	
            pub.publish(pos)
            if(rospy.wait_for_message('motion_over_topic', Coordinates)):
                rospy.set_param('dog/x', pos.x)
                rospy.set_param('dog/y', pos.y)
                #time.sleep(2 / sim_scale)
                if sleep_timer == 0:
                    return 'go_sleep'
            self.rate.sleep
        return 'go_play'

    ## Normal state callback that prints a string once the random target
    # position has been reached
    def normal_callback_motion(self, data):
        if rospy.get_param('state') == 'normal':
            rospy.loginfo('dog: %i %i position reached!', data.x, data.y)

    ## Normal state callback that prints a string once the robot acknowledges
    # the user's <<play>> request
    #def normal_callback_play(self, data):
    #    global playtime
    #    if rospy.get_param('state') == 'normal':        
    #        rospy.loginfo('dog: Ok, let\'s play!')
    #        playtime = 1

## Play state definition
class Play(smach.State):
    ## Play state initialization: set the outcomes and subscribe to the
    # gesture_topic and the motion_over_topic topics
    def __init__(self):
        # initialisation function, it should not wait
        smach.State.__init__(self, 
                             outcomes=['game_over'])
        rospy.Subscriber('motion_over_topic', Coordinates, self.play_callback_motion)
        #rospy.Subscriber('gesture_topic', Coordinates, self.play_callback_gesture)

    ## Play state execution: the robot reaches the users, then goes to the
    # pointed location, then comes back to the user and so on. After some time
    # it gets back to the Normal state
    def execute(self, userdata):
        # function called when exiting from the node, it can be blocking
        global gestured_point
        normal_timer = random.randint(4, 6)
        self.rate = rospy.Rate(200)
        rospy.set_param('state', 'play')
        pos = Coordinates()
        pub = rospy.Publisher('control_topic', Coordinates, queue_size=10)
        rospy.loginfo('dog: I am coming to you!')
        pos.x = rospy.get_param('person/x')
        pos.y = rospy.get_param('person/y')        
        pub.publish(pos)
        rospy.wait_for_message('motion_over_topic', Coordinates)
        rospy.set_param('dog/x', pos.x)
        rospy.set_param('dog/y', pos.y)
        while (normal_timer != 0 and not rospy.is_shutdown() and \
            rospy.get_param('state') == 'play'):
            normal_timer = normal_timer - 1
            #rospy.wait_for_message('gesture_topic', Coordinates)
            rospy.wait_for_message('motion_over_topic', Coordinates)
            rospy.set_param('dog/x', gestured_point.x)
            rospy.set_param('dog/y', gestured_point.y)
            time.sleep(3 / sim_scale)
            rospy.loginfo('dog: I am coming to you!')
            pos.x = rospy.get_param('person/x')
            pos.y = rospy.get_param('person/y')        
            pub.publish(pos)
            rospy.wait_for_message('motion_over_topic', Coordinates)
            rospy.set_param('dog/x', pos.x)
            rospy.set_param('dog/y', pos.y)
            self.rate.sleep
        if normal_timer == 0:
            return 'game_over'

    ## Play state callback that prints a string once the user
    # position has been reached
    def play_callback_motion(self, data):
        if rospy.get_param('state') == 'play':
            if data.x == rospy.get_param('person/x') and \
             data.y == rospy.get_param('person/y'):
                rospy.loginfo('dog: user position reached!')
            else:
                rospy.loginfo('dog: pointed %i %i position reached!', data.x, data.y)

    ## Play state callback that prints a string informing that the robot is
    # heading towards the pointed location, and then also prints a string when
    # that location has been reached
    #def play_callback_gesture(self, data):
    #    global gestured_point
    #    if rospy.get_param('state') == 'play':
    #        rospy.loginfo('dog: I am moving to gestured %i %i', data.x, data.y)
    #        pub = rospy.Publisher('control_topic', Coordinates, queue_size=10)
    #        pub.publish(data)
    #        gestured_point.x = data.x
    #        gestured_point.y = data.y

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