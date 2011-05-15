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
import Strat_Uninitialisation

from Inputs import Inputs
from Data import Data

###########################  TEMPORAL BEHAVIOR

class StratNode():
    
    def __init__(self):
        
        #creation of the node
        rospy.init_node('StratNode')
        #creation of the cadencer for all states
        Data.stateMachineRate =rospy.Rate(1)
        #linking of the input in Inputs to the topics
        Inputs.link()
        #creation of statemachine
        sm=MainStateMachine()
    
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
    
    
    
    
############################## STATE MACHINE
class MainStateMachine(smach.StateMachine):
    def __init__(self):
        smach.StateMachine.__init__(self,outcomes=['end'])
        with self:
            smach.StateMachine.add('Initialisation', Strat_Initialisation.Initialisation(),
                                   transitions={'endInitialisation':'StartSequence'})
            smach.StateMachine.add('StartSequence', Strat_StartSequence.StartSequence(),
                                   transitions={'gogogo':'Opening','problem':'end'})
            smach.StateMachine.add('Opening', Strat_Opening.Opening(),
                                    transitions={'endOpening':'Middlegame','problem':'Middlegame'})
            smach.StateMachine.add('Middlegame', Strat_Middlegame.Middlegame(),
                                    transitions={'endMiddlegame':'Endgame'})
            smach.StateMachine.add('Endgame', Strat_Endgame.Endgame(),
                                    transitions={'endEndgame':'Uninitialisation'})
            smach.StateMachine.add('Uninitialisation', Strat_Uninitialisation.Uninitialisation(),
                                    transitions={'endUninitialisation':'end'})


   
########################## EXECUTABLE 
#shall be always at the end ! so that every function is defined before
# main function, called by ros
if __name__ == '__main__':
    try:
        StratNode()
    except rospy.ROSInterruptException: pass