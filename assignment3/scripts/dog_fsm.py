#!/usr/bin/env python

## @package dog_fsm
# Emulates the robotic dog's finite state machine internal architecture. 
# The implemented states are Sleep, Normal, Play and Find

import roslib
import rospy
import smach
import smach_ros
import time
import random
import actionlib
import assignment3
import assignment3.msg
from std_msgs.msg import String
from assignment3.srv import BallService, Explore
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

## Acquire the list of available rooms from launch file
room_list = rospy.get_param('room_list')

## Acquire simulation speed scaling factor from launch file
sim_scale = rospy.get_param('sim_scale')

## variable used to let the fsm behave differently for the very first state only
first_iteration = 1

## variable used to state whether it is time to play or not
playtime = 0

## variable used to identify and store the requested room (hence ball) by the human
# when it equals 100, it is set to its default value, and does not correspond to
# any ball
play_ball_request = 100

## variable used to keep track of the robot's simulated battery charge
energy_timer = random.randint(4, 7)



## Sleep state definition
class Sleep(smach.State):
    ## Sleep state initialization: set the outcomes
    def __init__(self):
        # initialisation function, it should not wait
        smach.State.__init__(self, 
                             outcomes=['wake_up'])

    ## Sleep state execution: the robot goes to sleep once he gets back home.
    # It relies on the move_base algorithm in order to move in the environment,
    # and it does so by implementing an action client and asking it to reach
    # the coordinates corresponding to home
    def execute(self, userdata):
        # function called when exiting from the node, it can be blocking
        global first_iteration, energy_timer
        rospy.set_param('state','sleep')
        self.rate = rospy.Rate(200)
        ## move_base client used to reach home position
        mb_sleep_client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        home_pos = MoveBaseGoal()
        home_pos.target_pose.pose.orientation.w = 1.0
        home_pos.target_pose.header.frame_id = "map"
        home_pos.target_pose.header.stamp = rospy.Time.now()
        # set target as the home position (x-axis)
        home_pos.target_pose.pose.position.x = home_x
        # set target as the home position (y-axis)
        home_pos.target_pose.pose.position.y = home_y
        if first_iteration == 0:
            time.sleep(5) # wait for other nodes to launch
            rospy.loginfo('Dog: I am going to spleep!')
            mb_sleep_client.send_goal(home_pos)
            mb_sleep_client.wait_for_result()
            rospy.loginfo('Dog: home position reached!')
        else:
            first_iteration = 0    
        time.sleep(random.randint(5, 10) / sim_scale) # the dog is sleeping
        energy_timer = random.randint(2, 7)
        rospy.loginfo('Dog: Good morning!')
        return 'wake_up'



## Normal state definition
class Normal(smach.State):
    ## Normal state initialization: set the outcomes and subscribe to the
    # 'play_topic' topic, on which human.py publishes its commands
    def __init__(self):
        # initialisation function, it should not wait
        smach.State.__init__(self, 
                             outcomes=['go_play','go_sleep'])
        ## subscribed topic, used to receive commands from the human.py node
        rospy.Subscriber('play_topic', String, self.normal_callback)
    
    ## Normal state execution: the robot wanders randomly, by feeding
    # the move_base algorithm with randomly generated positions. If, while moving around,
    # the robot detects a new room/ball, it reaches it and stores the corresponding
    # position. At any time, a play command can be received by the human: if so happens,
    # the robotic dog transitions to the Play state. Every newly detected room,
    # along with its subsequent data collection process, consumes robot battery.
    # If the battery gets depleted, the robot goes to sleep, by transitioning
    # to the Sleep state.
    def execute(self, userdata):
        # function called when exiting from the node, it can be blocking
        global playtime, energy_timer
        rospy.set_param('state', 'normal')
        self.rate = rospy.Rate(200)
        ## move_base client used to send random positions as goals
        mb_normal_client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        goal_pos = MoveBaseGoal()
        goal_pos.target_pose.pose.orientation.w = 1.0
        while (energy_timer != 0 and playtime == 0 and (not rospy.is_shutdown()) and \
            rospy.get_param('state') == 'normal'):
            goal_pos.target_pose.header.frame_id = "map"
            goal_pos.target_pose.header.stamp = rospy.Time.now()
            goal_pos.target_pose.pose.position.x = random.randint(map_x_min, map_x_max)
            goal_pos.target_pose.pose.position.y = random.randint(map_y_min, map_y_max)
            mb_normal_client.send_goal(goal_pos)
            rospy.loginfo('Dog: I am moving to %i %i', \
                 goal_pos.target_pose.pose.position.x, goal_pos.target_pose.pose.position.y)
            mb_normal_client.wait_for_result()
            if(rospy.get_param('new_ball_detected') == 1):
                rospy.loginfo('Dog: I have spotted a new room!')
            while(rospy.get_param('new_ball_detected') == 1):
                self.rate.sleep
            energy_timer = energy_timer - 1	
            self.rate.sleep

        if energy_timer == 0:
            rospy.loginfo('Dog: I am going to sleep!')
            return 'go_sleep'

        elif playtime == 1:
            return 'go_play'

    ## Normal state callback that prints a string acknowledging that the robotic dog
    # has received a play request
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
    # 'play_topic' topic, on which human.py publishes its commands
    def __init__(self):
        # initialisation function, it should not wait
        smach.State.__init__(self, 
                             outcomes=['game_over','go_find'])
        ## subscribed topic, used to receive commands from the human.py node                    
        rospy.Subscriber('play_topic', String, self.play_callback)

    ## Play state execution: using the move_base algorithm, the robotic dog, gets
    # back home (i.e. close to the human), then starts listening for the
    # room request. Once received, it checks via a service implemented in ball_server.py
    # which are the corresponing ball coordinates to reach. If the ball is not yet in the
    # dog's database, it shifts to the Find state. If the ball has been seen before,
    # the dog starts moving to the chosen room, still relying on move_base. After
    # reaching that position, it comes back to the user, again via move_base:
    # whenever this process is completed, some battery is consumed. Furthermore,
    # I assumed that a single cycle of this state consumes slightly more battery,
    # on average, than one of the Normal state; for this reason, the threshold of
    # remaining battery before going back to the Normal state is higher than in the
    # previous case (i.e. from Normal to Sleep): this also allows me to make sure
    # that the robot can go to sleep before completely depleting its battery
    def execute(self, userdata):
        # function called when exiting from the node, it can be blocking
        global play_ball_request, energy_timer
        rospy.set_param('state', 'play')
        self.rate = rospy.Rate(200)
        ## move_base client that is used both to reach home location and the
        # room requested by the human
        mb_play_client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        target_pos = MoveBaseGoal()
        target_pos.target_pose.header.frame_id = "map"
        target_pos.target_pose.header.stamp = rospy.Time.now()
        # set target as the home position (x-axis)
        target_pos.target_pose.pose.position.x = home_x
        # set target as the home position (y-axis)
        target_pos.target_pose.pose.position.y = home_y
        rospy.set_param('play_task_status', 0)
        mb_play_client.send_goal(target_pos)
        mb_play_client.wait_for_result()
        rospy.loginfo('Dog: I reached your position')
        rospy.set_param('play_task_status', 1)
        while ((not rospy.is_shutdown()) and energy_timer != 1 \
                        and rospy.get_param('state') == 'play'):
            rospy.set_param('play_task_status', 0)
            while(play_ball_request == 100):
                self.rate.sleep
            rospy.wait_for_service('BallService')
            ## BallService client that asks for the coordinates of the ball
            # whose color matches the room requested by the human
            ball_service_client = rospy.ServiceProxy('BallService', BallService)
            ball_location = ball_service_client(play_ball_request)
            temp_unknown_ball = play_ball_request
            play_ball_request = 100
            if (ball_location.x != 100 and ball_location.y != 100):
                target_pos.target_pose.pose.orientation.w = 1.0
                target_pos.target_pose.header.frame_id = "map"
                target_pos.target_pose.header.stamp = rospy.Time.now()
                target_pos.target_pose.pose.position.x = ball_location.x
                target_pos.target_pose.pose.position.y = ball_location.y
                mb_play_client.send_goal(target_pos)
                mb_play_client.wait_for_result()
                rospy.loginfo('Dog: I got to the room')
                target_pos.target_pose.header.frame_id = "map"
                target_pos.target_pose.header.stamp = rospy.Time.now()
                # set target as home position (x-axis)
                target_pos.target_pose.pose.position.x = home_x
                # set target as home position (y-axis)
                target_pos.target_pose.pose.position.y = home_y
                mb_play_client.send_goal(target_pos)
                mb_play_client.wait_for_result()
                rospy.loginfo('Dog: I am finally back to you')
                rospy.set_param('play_task_status', 2)
                energy_timer = energy_timer - 1
            else:
                rospy.set_param('unknown_ball', temp_unknown_ball)
                break
            self.rate.sleep

        if energy_timer == 1:
            rospy.loginfo('Dog: I am too tired to play any longer: ' + \
                             'I will briefly go in Normal')
            return 'game_over'

        elif rospy.get_param('unknown_ball') != 100:
            rospy.loginfo('Dog: I don\'t know where the %s is. ' + \
                             'I\'ll search around for it' + \
                             room_list[rospy.get_param('unknown_ball')])
            return 'go_find'

    ## Play state callback that is used to translate the room ordered
    # by the human into the ball representing that same room, and that will then be
    # reached or searched by the robotic dog
    def play_callback(self, data):
        global play_ball_request
        if (rospy.get_param('state') == 'play'):
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
    ## Find state initialization: set the outcomes
    def __init__(self):
        # initialisation function, it should not wait
        smach.State.__init__(self, 
                             outcomes=['find_over'])

    ## Find state execution: in this state, the robotic dog looks for the goal ball,
    # determined in the Play state. In order to do so, this state relies on the
    # explore_lite algorithm; a service client sends a flag to the server, which
    # corresponds to the signal the server itself is waiting it order to run the
    # above mentioned algorithm. The code of the algorithm, which is contained inside
    # the Explore class (see explore.cpp), has been adjusted with the inclusion of
    # a method able to handle requests coming from this node. The exploration
    # algorithm keeps running until dog_vision.py stops its execution, i.e.
    # a new ball has been found. If the new ball is indeed the goal one,
    # the robot eventually gets back to the Play state. The human will then immediately
    # call the dog back to its position, and another Play state cycle can begin
    def execute(self, userdata):
        # function called when exiting from the node, it can be blocking
        rospy.set_param('state', 'find')
        self.rate = rospy.Rate(200)
        ## explore_lite service client that lets the algorithm start
        # exploring the robotic dog's surroundings
        rospy.wait_for_service('explore_start_service')
        explore_start = rospy.ServiceProxy('explore_start_service', Explore)
        explore_start(1)
        rospy.loginfo('Dog: Exploration started')
        while ((not rospy.is_shutdown()) and rospy.get_param('state') == 'find' \
             and rospy.get_param('unknown_ball') != 100):
            self.rate.sleep
        rospy.loginfo('Dog: I found the room you asked for!')
        rospy.set_param('play_task_status', 2)
        return 'find_over'



## Finite state machine's (fsm) main. It initializes the dog_fsm_node and setups
# a SMACH state machine along with all the 4 possible states
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
        smach.StateMachine.add('FIND', Find(), 
                               transitions={'find_over':'PLAY'})

    # Create and start the introspection server for visualization
    sis = smach_ros.IntrospectionServer('server_name', sm, '/SM_ROOT')
    sis.start()

    # outcome = sm.execute() # an output variable is to be used if
                             # this finite state machine is nested
                             # inside another one

    # Execute the state machine
    sm.execute()

    # Wait for ctrl-c to stop the application
    rospy.spin()
    sis.stop()

if __name__ == '__main__':
    main()