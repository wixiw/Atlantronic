#!/usr/bin/env python

#libraries for ROS
import roslib; roslib.load_manifest('arp_master')

from arp_master import *

#import the main state machines substates     
from a2_opening import Strat_Opening
from a3_middleGame import Strat_MiddleGame
from a4_endGame import Strat_EndGame

from arp_master.strat_2012 import *

###########################  TEMPORAL BEHAVIOR

class StratNode_vierge():
    
    def __init__(self):
        
        #creation of the node
        rospy.init_node('StratNode')
        #recuperation des parametres (on a besoin d'etre un noeud pour ca
        Table2012.getParams()
        Robot2012.getParams()
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
        #sis = smach_ros.IntrospectionServer('strat_server', sm, '/SratNode_vierge')
        #sis.start()
        sm.execute()
        
        #sis.stop()
    
############################## MAIN STATE MACHINE CREATION
class MainStateMachine(smach.StateMachine):
    def __init__(self):
        smach.StateMachine.__init__(self,outcomes=['end'])
        with self:
            smach.StateMachine.add('Initialisation', 
                                   InitStates.Initialisation2012(),
                                   transitions={'endInitialisation':'StartSequence', 'failed':'end'})
            smach.StateMachine.add('StartSequence', 
                                   InitStates.StartSequence2012(1.140 + Robot2012.CDG_POSE.x,0.75,-pi),
                                   transitions={'gogogo':'Opening','problem':'end'})
            smach.StateMachine.add('Opening', 
                                   Strat_Opening.Opening(),
                                    transitions={'endOpening':'MiddleGame','problem':'Uninitialisation'})
            smach.StateMachine.add('MiddleGame', 
                                   Strat_MiddleGame.MiddleGame(),
                                    transitions={'endMiddleGame':'EndGame'})
            smach.StateMachine.add('EndGame', 
                                   Strat_EndGame.EndGame(),
                                    transitions={'endEndGame':'Uninitialisation'})
            smach.StateMachine.add('Uninitialisation', 
                                   InitStates.Uninitialisation2012(1.140,0.75,-pi),
                                    transitions={'endUninitialisation':'end'})

   
########################## EXECUTABLE 
#shall be always at the end ! so that every function is defined before
# this main function is the one called by ros
if __name__ == '__main__':
    try:
        StratNode_vierge()
    except rospy.ROSInterruptException: 
        rospy.loginfo("handling rospy.ROSInterruptException ...")
        rospy.loginfo("Exiting")
        pass