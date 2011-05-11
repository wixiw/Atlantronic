#!/usr/bin/env python
import roslib; roslib.load_manifest('arp_master')
import rospy
# library for the state machine
import smach
import smach_ros
import smach_msgs
# import the definition of the messages
from arp_core.msg import Obstacle
from arp_core.msg import StartColor
from arp_core.msg import Start
#import the other strat modules    
import CyclicState
import Strat_Initialisation
import Strat_StartSequence
import Strat_Opening
import Strat_Middlegame
import Strat_Endgame
import Strat_Unitialisation

from Inputs import Inputs

###########################  TEMPORAL BEHAVIOR

class StratNode():
    
    def __init__(self):
        
        rospy.init_node('StratNode')
        stateMachineRate =rospy.Rate(1)
        
        Inputs.link()
        #creation of statemachine
        sm=MainStateMachine()
        #sis = smach_ros.IntrospectionServer('server_name', sm, '/SM_ROOT')
        #sis.start()
    
        #welcome message
        rospy.loginfo("******************************************************")
        rospy.loginfo("Welcome to Advanced Robotics Platform. I am StratNode.")
        rospy.loginfo("Choose color with button")
        rospy.loginfo("Then plug start")
        rospy.loginfo("Wait for initialisation sequence")
        rospy.loginfo("And unplug start")
        rospy.loginfo("******************************************************")
    
        # initialise the smach introspection server to view the state machine with :
        #  rosrun smach_viewer smach_viewer.py
        sis = smach_ros.IntrospectionServer('strat_server', sm, '/SratNode')
        sis.start()
        sm.execute()
        
        sis.stop()
    
    
############################# MAIN LOOP
def mainloop():
    global obstacle
    rospy.loginfo("obstacle: %i"%obstacle.data.detected)
    rospy.loginfo("callback appelee: %i fois"%obstacle.ncall)
    
    
############################## STATE MACHINE
class MainStateMachine(smach.StateMachine):
    def __init__(self):
        smach.StateMachine.__init__(self,outcomes=['end'])
        with self:
            smach.StateMachine.add('Initialisation', Strat_Initialisation.Initialisation(),
                                   transitions={'endInitialisation':'StartSequence'})
            smach.StateMachine.add('StartSequence', Strat_StartSequence.StartSequence(),
                                   transitions={'gogogo':'end'})


   
########################## EXECUTABLE 
#shall be always at the end ! so that every function is defined before
# main function, called by ros
if __name__ == '__main__':
    try:
        StratNode()
    except rospy.ROSInterruptException: pass