#!/usr/bin/env python

import roslib
import rospy
import smach
import smach_ros
import time
import random
from std_msgs.msg import String

#motion_check = 1

# define state Sleep
class Sleep(smach.State):
    def __init__(self):
        # initialisation function, it should not wait
        smach.State.__init__(self, 
                             outcomes=['wake_up'],
                             input_keys=['fsm_robot_in', 'fsm_home_in'],
                             output_keys=['fsm_robot_out'])
        #global sub
        #sub = rospy.Subscriber('motion_over_topic', String, self.sleep_callback)
        rospy.Subscriber('motion_over_topic', String, self.sleep_callback)

    def execute(self, userdata):
        # function called when exiting from the node, it can be blocking
        userdata.fsm_robot_out = userdata.fsm_home_in # reach home position
        rospy.loginfo('MiRo: I am going to spleep!')
        pub = rospy.Publisher('control_topic', String, queue_size=10)
        pub.publish("home position")
        time.sleep(random.randint(1, 5)) # MiRo is sleeping
        rospy.loginfo('MiRo: Good morning!')
        return 'wake_up'

    def sleep_callback(self, data):
        rospy.loginfo('MiRo: home position reached!')
        #sub.unregister()


# define state Normal
class Normal(smach.State):
    def __init__(self):
        # initialisation function, it should not wait
        smach.State.__init__(self, 
                             outcomes=['go_play','go_sleep'],
                             input_keys=['fsm_robot_in'],
                             output_keys=['fsm_robot_out'])
        #sub = rospy.Subscriber('motion_over_topic', String, self.normal_callback)
        rospy.Subscriber('motion_over_topic', String, self.normal_callback)
        #sub2 = rospy.Subscriber('play_topic', String, self.normal_callback_2)
        #rospy.Subscriber('play_topic', String, self.normal_callback_2)
        
    def execute(self, userdata):
        # function called when exiting from the node, it can be blocking
        sleep_timer = random.randint(2, 10)
        self.rate = rospy.Rate(200) #200
        motion_check = 1
        while (sleep_timer != 0 and not rospy.is_shutdown()):
            if motion_check == 1:
                sleep_timer = sleep_timer - 1
                new_pos = [random.randint(0, 10), random.randint(0, 10)] # acquire new pos
                userdata.fsm_robot_out = new_pos #e un problema se vengo interrotto tra que e sopra
                position = ' '.join([str(elem) for elem in new_pos]) 
                rospy.loginfo('MiRo: I am moving to %s' %position)
                userdata.fsm_robot_out = new_pos
                pub = rospy.Publisher('control_topic', String, queue_size=10)
                pub.publish(position)
                #rospy.Subscriber('motion_over_topic', String, self.normal_callback)
                time.sleep(2)
                #rospy.Subscriber('play_topic', String, self.normal_callback_2)
            if sleep_timer == 0:
                return 'go_sleep'
            self.rate.sleep

    def normal_callback(self, data):
        rospy.loginfo('MiRo: target position reached!')
        self.motion_check = 0
        #sub.unregister()

    def normal_callback_2(self, data):
        rospy.loginfo('MiRo: let\'s play')
        return 'go_play'
        #sub2.unregister()


# define state Play
class Play(smach.State):
    def __init__(self):
        # initialisation function, it should not wait
        smach.State.__init__(self, 
                             outcomes=['go_sleep','game_over'],
                             input_keys=['fsm_robot_in', 'fsm_user_in'],
                             output_keys=['fsm_robot_out'])
        
    def execute(self, userdata):
        # function called when exiting from the node, it can be blocking
        # reach user position
        # ask, then reach pointed or told position
        # do it for some time
        # in the end go to normal,but also here we need sleep timer too
        rospy.loginfo('MiRo: I am going to spleep!')
        time.sleep(random.randint(1, 5))
        return 'go_sleep'

        
def main():
    rospy.init_node('miro_fsm_node')

    # Create a SMACH state machine
    sm = smach.StateMachine(outcomes=['container_interface'])
    sm.userdata.fsm_robot = [0, 0] # current robot position
    sm.userdata.fsm_home = [3, 6] # home position
    sm.userdata.fsm_user = [4, 4] # user position
    sm.userdata.fsm_gridsize = [10, 10] # grid sizes    

    # Open the container
    with sm:
        # Add states to the container
        smach.StateMachine.add('SLEEP', Sleep(), 
                               transitions={'wake_up':'NORMAL'},
                               remapping={'fsm_robot_in':'fsm_robot', 
                                          'fsm_robot_out':'fsm_robot',
                                          'fsm_home_in':'fsm_home'})
        smach.StateMachine.add('NORMAL', Normal(), 
                               transitions={'go_play':'PLAY', 
                                            'go_sleep':'SLEEP'},
                               remapping={'fsm_robot_in':'fsm_robot', 
                                          'fsm_robot_out':'fsm_robot'})
        smach.StateMachine.add('PLAY', Play(), 
                               transitions={'go_sleep':'SLEEP', 
                                            'game_over':'NORMAL'},
                               remapping={'fsm_robot_in':'fsm_robot',
                                          'fsm_user_in':'fsm_user',
                                          'fsm_robot_out':'fsm_robot'})


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