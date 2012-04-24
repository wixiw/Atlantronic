#!/usr/bin/env python

#libraries for ROS
import roslib; roslib.load_manifest('arp_master')

from arp_master import *

#import the main state machines substates     
from Strat_Initialisation import Strat_Initialisation
from Strat_StartSequence import Strat_StartSequence
from Strat_Opening import Strat_Opening
from Strat_Middlegame import Strat_Middlegame
from Strat_Endgame import Strat_Endgame
from Strat_Uninitialisation import Strat_Uninitialisation


###########################  TEMPORAL BEHAVIOR

class StratNode_vierge():
    
    def __init__(self):
        
        #creation of the node
        rospy.init_node('StratNode')
        # recuperation des parametres
        Table.init()
        Robot.init()
        #creation of the cadencer for all states
        Data.stateMachineRate =rospy.Rate(10)
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
        sis = smach_ros.IntrospectionServer('strat_server', sm, '/SratNode_vierge')
        sis.start()
        sm.execute()
        
        sis.stop()
    
############################## MAIN STATE MACHINE CREATION
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
# this main function is the one called by ros
if __name__ == '__main__':
    try:
        StratNode_vierge()
    except rospy.ROSInterruptException: pass