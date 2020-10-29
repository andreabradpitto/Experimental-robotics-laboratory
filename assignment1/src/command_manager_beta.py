#!/usr/bin/env python

import roslib
import rospy
import smach
import smach_ros
import time
import random
from std_msgs.msg import String
from assignment1.msg import Coordinates

## Acquire map size parameters from launch file
map_x_max = rospy.get_param("map/x_max")
map_y_max = rospy.get_param("map/y_max")

## Acquire home position parameters from launch file
home_x = rospy.get_param("home/x")
home_y = rospy.get_param("home/y")

## Acquire person's position parameters from launch file
person_x = rospy.get_param("person/x")
person_y = rospy.get_param("person/y")

miro_state = 0 # 0 = sleep, 1 = normal, 2 = play
sleep_flag = True

# define state Sleep
class Sleep(smach.State):
    def __init__(self):
        # initialisation function, it should not wait
        smach.State.__init__(self, 
                             outcomes=['wake_up'],
                             input_keys=['x_in','y_in','home_x_in','home_y_in'],
                             output_keys=['x_out','y_out'])
        rospy.Subscriber('motion_over_topic', Coordinates, self.sleep_callback)

    def execute(self, userdata):
        # function called when exiting from the node, it can be blocking
        self.rate = rospy.Rate(200) #200
        global miro_state
        miro_state = 0
        global sleep_flag
        sleep_flag = True
        pos = Coordinates()
        pos.x = userdata.home_x_in # reach home position
        pos.y = userdata.home_y_in # reach home position
        rospy.loginfo('MiRo: I am going to spleep!')
        userdata.x_out = pos.x
        userdata.y_out = pos.y
        pub = rospy.Publisher('control_topic', Coordinates, queue_size=10)
        pub.publish(pos)
        while (sleep_flag and not rospy.is_shutdown()):
            rospy.wait_for_message('motion_over_topic', Coordinates)
            #time.sleep(random.randint(1, 5)) # MiRo is sleeping
            #rospy.loginfo('MiRo: Good morning!')
            self.rate.sleep
        return 'wake_up'

    def sleep_callback(self, data):
        global miro_state
        global sleep_flag
        if miro_state == 0:
            rospy.loginfo('MiRo: home position reached!')
            time.sleep(random.randint(1, 5)) # MiRo is sleeping
            rospy.loginfo('MiRo: Good morning!')
            sleep_flag = False


# define state Normal
class Normal(smach.State):
    def __init__(self):
        # initialisation function, it should not wait
        smach.State.__init__(self, 
                             outcomes=['go_play','go_sleep'],
                             input_keys=['x_in','y_in','map_x_max_in','map_y_max_in'],
                             output_keys=['x_out','y_out'])
        rospy.Subscriber('motion_over_topic', Coordinates, self.normal_callback)
        #rospy.Subscriber('play_topic', Coordinates, self.normal_callback_2)
        
    def execute(self, userdata):
        # function called when exiting from the node, it can be blocking
        sleep_timer = random.randint(2, 10)
        self.rate = rospy.Rate(200) #200
        global miro_state
        miro_state = 1
        pos = Coordinates()
        while (sleep_timer != 0 and not rospy.is_shutdown()):
            sleep_timer = sleep_timer - 1     
            pos.x = random.randint(0, userdata.map_x_max_in)
            pos.y = random.randint(0, userdata.map_y_max_in)
            userdata.x_out = pos.x #e un problema se vengo interrotto tra que e sopra
            userdata.y_out = pos.y
            #position = ' '.join([str(elem) for elem in pos]) 
            rospy.loginfo('MiRo: I am moving to %i %i', pos.x, pos.y)	
            pub = rospy.Publisher('control_topic', Coordinates, queue_size=10)
            pub.publish(pos)
            rospy.wait_for_message('motion_over_topic', Coordinates)
            if sleep_timer == 0:
                return 'go_sleep'
            self.rate.sleep

    def normal_callback(self, data):
        global miro_state
        if miro_state == 1:
            rospy.loginfo('MiRo: target position reached!')

    def normal_callback_2(self, data):
        global miro_state
        if miro_state == 1:        
            rospy.loginfo('MiRo: let\'s play')
        return 'go_play'


# define state Play
class Play(smach.State):
    def __init__(self):
        # initialisation function, it should not wait
        smach.State.__init__(self, 
                             outcomes=['go_sleep','game_over'],
                             input_keys=['x_in','y_in','person_x_in','person_y_in'],
                             output_keys=['x_out','y_out'])
        
    def execute(self, userdata):
        # function called when exiting from the node, it can be blocking
        # reach user position
        # ask, then reach pointed or told position
        # do it for some time
        # in the end go to normal,but also here we need sleep timer too
        miro_state = 2
        rospy.loginfo('MiRo: I am going to spleep!')
        time.sleep(random.randint(1, 5))
        return 'go_sleep'

        
def main():
    rospy.init_node('miro_fsm_node')

    # Create a SMACH state machine
    sm = smach.StateMachine(outcomes=['container_interface'])
    sm.userdata.x = 0 # current robot position
    sm.userdata.y = 0 # current robot position
    sm.userdata.map_x_max = map_x_max # grid size
    sm.userdata.map_y_max = map_y_max # grid size
    sm.userdata.home_x = home_x # home position
    sm.userdata.home_y = home_y # home position
    sm.userdata.person_x = person_x # user position
    sm.userdata.person_y = person_y # user position

    # Open the container
    with sm:
        # Add states to the container
        smach.StateMachine.add('SLEEP', Sleep(), 
                               transitions={'wake_up':'NORMAL'},
                               remapping={'x_in':'x', 
                                          'x_out':'x',
                                          'y_in':'y',
                                          'y_out':'y',
                                          'home_x_in':'home_x',
                                          'home_y_in':'home_y'})
        smach.StateMachine.add('NORMAL', Normal(), 
                               transitions={'go_play':'PLAY', 
                                            'go_sleep':'SLEEP'},
                               remapping={'x_in':'x', 
                                          'x_out':'x',
                                          'y_in':'y',
                                          'y_out':'y',
                                          'map_x_max_in':'map_x_max',
                                          'map_y_max_in':'map_y_max'})
        smach.StateMachine.add('PLAY', Play(), 
                               transitions={'go_sleep':'SLEEP', 
                                            'game_over':'NORMAL'},
                               remapping={'x_in':'x', 
                                          'x_out':'x',
                                          'y_in':'y',
                                          'y_out':'y',
                                          'person_x_in':'person_x',
                                          'person_y_in':'person_y'})


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