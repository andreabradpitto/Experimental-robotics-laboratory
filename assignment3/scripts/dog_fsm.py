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
import assignment3
import actionlib
from std_msgs.msg import String
from assignment3.msg import Coordinates
from assignment3.srv import BallService # magari inutile (anche quello sopra) visto che
                                        # ho import assignment3
from assignment3.action import IntAction
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from actionlib_msgs.msg import GoalStatus
from nav_msgs.msg import Odometry

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

room_list = rospy.get_param('room_list')
#room_list = ['entrance', 'closet', 'livingroom', 'kitchen', 'bathroom', 'bedroom']

## Acquire simulation speed scaling factor from launch file
sim_scale = rospy.get_param('sim_scale')

## variable used to let the fsm behave differently for the very first state only
first_iteration = 1

## variable used to state whether it is time to play or not
playtime = 0

play_ball_request = 100

energy_timer = random.randint(4, 7)



## Sleep state definition
class Sleep(smach.State):
    ## Sleep state initialization: set the outcomes and subscribe to the
    # motion_over_topic topic
    def __init__(self):
        # initialisation function, it should not wait
        smach.State.__init__(self, 
                             outcomes=['wake_up'])

    ## Sleep state execution: the robot goes to sleep once he gets back home
    def execute(self, userdata):
        # function called when exiting from the node, it can be blocking
        global first_iteration, energy_timer
        rospy.set_param('state','sleep')
        self.rate = rospy.Rate(200)
        #pos = Coordinates()
        mb_sleep_client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        home_pos = MoveBaseGoal()
        #home_pos.target_pose.pose.orientation.w = 1.0
        home_pos.target_pose.header.frame_id = "map"
        home_pos.target_pose.header.stamp = rospy.Time.now()
        home_pos.target_pose.pose.position.x = home_x # set target as the home position (x-axis)
        home_pos.target_pose.pose.position.y = home_y # set target as the home position (y-axis)
        #pos.x = home_x # set target as the home position (x-axis)
        #pos.y = home_y # set target as the home position (y-axis)
        #pub_sleep = rospy.Publisher('control_topic', Coordinates, queue_size=10)
        if first_iteration == 0:
            rospy.loginfo('Dog: I am going to spleep!')
            time.sleep(random.randint(2, 5) / sim_scale)
        else:
            time.sleep(random.randint(2, 5) / sim_scale) 
        #rospy.set_param('position_reached', 0)
        #pub_sleep.publish(pos)
        mb_sleep_client.send_goal(home_pos)
        mb_sleep_client.wait_for_result()
        #while(not rospy.get_param('position_reached')):
        #    self.rate.sleep()
        if (first_iteration == 0):
            rospy.loginfo('Dog: home position reached!')
            rospy.set_param('dog/x', home_x)
            rospy.set_param('dog/y', home_y)
        elif(rospy.get_param('state') == 'sleep'):
            rospy.set_param('dog/x', home_x)
            rospy.set_param('dog/y', home_y)
            first_iteration = 0    
        time.sleep(random.randint(2, 5) / sim_scale) # the dog is sleeping
        energy_timer = random.randint(2, 7)
        rospy.loginfo('Dog: Good morning!')
        return 'wake_up'



## Normal state definition
class Normal(smach.State):
    ## Normal state initialization: set the outcomes and subscribe to the
    # play_topic and the motion_over_topic topics
    def __init__(self):
        # initialisation function, it should not wait
        smach.State.__init__(self, 
                             outcomes=['go_play','go_sleep'])
        rospy.Subscriber('play_topic', String, self.normal_callback)
    
    ## Normal state execution: the robot wanders randomly, while waiting for
    # finding the ball, or becoming tired and then going to sleep
    def execute(self, userdata):
        # function called when exiting from the node, it can be blocking
        global playtime, energy_timer
        rospy.set_param('state', 'normal')
        self.rate = rospy.Rate(200)
        mb_normal_client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        goal_pos = MoveBaseGoal()
        #goal_pos.target_pose.pose.orientation.w = 1.0
        while (energy_timer != 0 and playtime == 0 and (not rospy.is_shutdown()) and \
            rospy.get_param('state') == 'normal'):
            goal_pos.target_pose.header.frame_id = "map"
            goal_pos.target_pose.header.stamp = rospy.Time.now()
            goal_pos.target_pose.pose.position.x = random.randint(map_x_min, map_x_max)
            goal_pos.target_pose.pose.position.y = random.randint(map_y_min, map_y_max)
            mb_normal_client.send_goal(goal_pos)
            rospy.loginfo('Dog: I am moving to %i %i', \
                 goal_pos.target_pose.pose.position.x, goal_pos.target_pose.pose.position.y)
            while(rospy.get_param('new_ball_detected') == 0):
                mb_normal_client.wait_for_result()
                #tecnicamente credo che cosi lui giri finche il dog non arriva a goal,
                #il goal e impossibile o una nuova palla viene individuata
                # se goal impossibile o raggiunto vado avanti subito direi
                #mentre se appare palla mi fermo a meta lavoro ma nell'if sotto cancello
                #il goal

                #controlla status goal, se succeeded printo e aggiorno posizione corretta
                #altrimenti faccio get param posizione corrente e la setto nel param server
                #questo è il caso caso in cui fallisce (goal impossibile)

            if(rospy.get_param('new_ball_detected') == 1):
                mb_normal_client.cancel_all_goals()
            #    qui ci va codice "track sub-state":
            #    il movimento meglio farlo gestire all'altro nodo
            #    devo settare nuova pos palla eccetera...
            while(rospy.get_param('new_ball_detected') == 1):
                self.rate.sleep # e ok cosi questo sleep?

            #pos = rospy.wait_for_message('odom', Odometry, timeout = None)
            #pos = Odometry() # DA COMMENTARE!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
            #rospy.set_param('dog/x', pos.pose.pose.position.x)
            #rospy.set_param('dog/y', pos.pose.pose.position.y)

            #scopri quale palla è stata scoperta e assegna la stanza corrispondente
            #forse meglio far gestire questo a play
            
            energy_timer = energy_timer - 1	
            self.rate.sleep

        if energy_timer == 0:
            rospy.loginfo('Dog: I am going to sleep!')
            return 'go_sleep'

        elif playtime == 1:
            #rospy.loginfo('Dog: Ok, let\'s play!')
            return 'go_play'

    ## Normal state callback that prints a string once the robotic dog
    # acknowledges the ball
    def normal_callback(self, data):
        global playtime
        if (rospy.get_param('state') == 'normal'):
            rospy.loginfo('Dog: I have received a play request! Woof!')
            mb_normal_client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
            mb_normal_client.cancel_all_goals()
            playtime = 1



## Play state definition
class Play(smach.State):
    ## Play state initialization: set the outcomes and subscribe to the
    # ball_detection and the motion_over_topic topics
    def __init__(self):
        # initialisation function, it should not wait
        smach.State.__init__(self, 
                             outcomes=['game_over'])
        rospy.Subscriber('play_topic', String, self.play_callback)

    ## Play state execution: the robotic dog follows the ball as long as
    # it is in his sight. If the ball is still, the dog starts turning
    # it head. If the ball is lost over a certain amount of time, the 
    # robot gets back to the Normal state
    def execute(self, userdata):
        # function called when exiting from the node, it can be blocking
        global play_ball_request, energy_timer
        rospy.set_param('state', 'play')
        self.rate = rospy.Rate(200)
        mb_play_client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        target_pos = MoveBaseGoal()
        #ball_location = BallService()
        target_pos.target_pose.header.frame_id = "map"
        target_pos.target_pose.header.stamp = rospy.Time.now()
        target_pos.target_pose.pose.position.x = home_x # set target as home position (x-axis)
        target_pos.target_pose.pose.position.y = home_y # set target as home position (y-axis)
        mb_play_client.send_goal(target_pos)
        mb_play_client.wait_for_result()
        rospy.loginfo('Dog: I reached your position')
        rospy.set_param('play_task_status', 1)
        #while ((not rospy.is_shutdown()) and energy_timer != 1 \
                        #and rospy.get_param('unknown_ball') == 100 and rospy.get_param('state') == 'play'):
        #NB ora tornero sempre in normal con una mossa disponibile: assumo che
        # il play state richieda piu batteria
        while ((not rospy.is_shutdown()) and energy_timer != 1 \
                        and rospy.get_param('state') == 'play'):
            rospy.set_param('play_task_status', 0)
            while(play_ball_request == 100):
                self.rate.sleep
            rospy.wait_for_service('BallService')
            ball_service_client = rospy.ServiceProxy('BallService', BallService)
            ball_location = ball_service_client(play_ball_request)
            temp_unknown_ball = play_ball_request
            play_ball_request = 100
            if (ball_location.x != 100 and ball_location.y != 100):
                #target_pos.target_pose.pose.orientation.w = 1.0
                target_pos.target_pose.header.frame_id = "map"
                target_pos.target_pose.header.stamp = rospy.Time.now()
                target_pos.target_pose.pose.position.x = ball_location.x
                target_pos.target_pose.pose.position.y = ball_location.y
                mb_play_client.send_goal(target_pos)
                mb_play_client.wait_for_result()
                rospy.loginfo('Dog: I got to the room')
                rospy.set_param('play_task_status', 2) # questo stato non serve ma ci sta
                target_pos.target_pose.header.frame_id = "map"
                target_pos.target_pose.header.stamp = rospy.Time.now()
                target_pos.target_pose.pose.position.x = home_x # set target as home position (x-axis)
                target_pos.target_pose.pose.position.y = home_y # set target as home position (y-axis)
                mb_play_client.send_goal(target_pos)
                mb_play_client.wait_for_result()
                rospy.loginfo('Dog: I am finally back to you')
                rospy.set_param('play_task_status', 3)
                energy_timer = energy_timer - 1
            else:
                rospy.set_param('unknown_ball', temp_unknown_ball)
                break
            self.rate.sleep

        if energy_timer == 1:
            rospy.loginfo('Dog: I am too tired to play any longer: I will briefly go in Normal')
            return 'game_over'

        elif rospy.get_param('unknown_ball') != 100:
            rospy.loginfo('Dog: I don\'t know where the %s is. I\'ll search around for it', \
                 room_list[rospy.get_param('unknown_ball')])
            return 'go_find'

    ## Play state callback: as the dog has lost the ball, order the
    # finite state machine to get back to the Normal state
    def play_callback(self, data):
        global play_ball_request
        if (rospy.get_param('state') == 'play'):
            #order_string = 'Dog: I will to GoTo the ' + data.data
            #rospy.loginfo(order_string)
            rospy.loginfo('Dog: I will try to go to the %s', data.data)
            if data.data == room_list[0]:
                play_ball_request = 0
            elif data.data == room_list[1]:
                play_ball_request = 1
            elif data.data == room_list[2]:
                play_ball_request = 2
            elif data.data == room_list[3]:
                play_ball_request = 3
            elif data.data == room_list[4]:
                play_ball_request = 4
            else:
                play_ball_request = 5



## Find state definition
class Find(smach.State):
    ## Find state initialization: set the outcomes and subscribe to the
    # ball_detection and the motion_over_topic topics
    def __init__(self):
        # initialisation function, it should not wait
        smach.State.__init__(self, 
                             outcomes=['game_over'])

    ## Find state execution: the robotic dog follows the ball as long as
    # it is in his sight. If the ball is still, the dog starts turning
    # it head. If the ball is lost over a certain amount of time, the 
    # robot gets back to the Play state
    def execute(self, userdata):
        # function called when exiting from the node, it can be blocking
        rospy.set_param('state', 'find')
        self.rate = rospy.Rate(200)
        explore_find_client = actionlib.SimpleActionClient('explore', IntAction)
        explore_find_client.send_goal(1)
        while ((not rospy.is_shutdown()) and rospy.get_param('state') == 'find' \
             and rospy.get_param('unknown_ball') != 100):
            self.rate.sleep
        rospy.loginfo('Dog: I found the room you asked for!')
        rospy.set_param('play_task_status', 3)
        return 'find_over'



## Finite state machine's (fsm) main. It initializes the dog_fsm_node and setups
# a SMACH state machine along with all the three possible states
def main():
    rospy.init_node('dog_fsm_node', anonymous = True)

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
                               transitions={'game_over':'NORMAL',
                                            'go_find':'FIND'})
        smach.StateMachine.add('FIND', Play(), 
                               transitions={'find_over':'PLAY'})

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