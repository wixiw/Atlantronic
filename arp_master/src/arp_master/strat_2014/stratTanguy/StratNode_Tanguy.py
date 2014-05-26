#!/usr/bin/env python
# -*- coding: latin-1 -*-

#libraries for ROS
import roslib; roslib.load_manifest('arp_master')

from arp_master import *

from arp_master.util import *
from arp_master.fsmFramework import *
from arp_master.commonStates import *
from arp_master.strat_2014 import *

#import the main state machines substates     
#from a0_initialisation import Strat_Initialisation => utilisation de l'etat commun
#from a1_startSequence import Strat_StartSequence => utilisation de l'etat commun
from a2_opening import Tanguy_Opening
#from a3_middleGame import Strat_MiddleGame => utilisation de l'etat commun
from a4_endGame import Tanguy_EndGame
#from a5_uninitialisation import Strat_Uninitialisation => utilisation de l'etat commun


###########################  TEMPORAL BEHAVIOR

class StratNode_Tanguy():
    
    def __init__(self):
        
        #creation of the node
        rospy.init_node('StratNode')
        #recuperation des parametres (on a besoin d'etre un noeud pour ca
        Table2014.getParams()
        #creation of the cadencer for all states
        Data.stateMachineRate =rospy.Rate(10)
        #linking of the input in Inputs to the topics
        Inputs.link()
        #creation of statemachine
        sm=MainStateMachine()
    
        #welcome message
        rospy.loginfo("******************************************************")
        rospy.loginfo("Welcome to Advanced Robotics Platform. I am StratNode 2014.")
        rospy.loginfo("Choose color with button")
        rospy.loginfo("Then plug start")
        rospy.loginfo("Wait for initialisation sequence")
        rospy.loginfo("And unplug start")
        rospy.loginfo("******************************************************")
    
        # initialise the smach introspection server to view the state machine with :
        #  rosrun smach_viewer smach_viewer.py
        sis = smach_ros.IntrospectionServer('strat_server', sm, '/StratNode_Tanguy')
        sis.start()
        sm.execute()
        
        sis.stop()
    
############################## MAIN STATE MACHINE CREATION
class MainStateMachine(smach.StateMachine):
    def __init__(self):
        smach.StateMachine.__init__(self,outcomes=['end'])
        with self:
            smach.StateMachine.add('Initialisation', 
                                   Strat_Initialisation.Initialisation(),
                                   transitions={'endInitialisation':'StartSequence','failed':'end'})

            smach.StateMachine.add('StartSequence',
# Uncomment either this line to perform the 2014 init sequence with recalOnBorders
#                                  InitStates2014.StartSequence2014(Table2014.P_START_POS),
# Either this line to start directly from the START_POS 
                                   Strat_StartSequence.StartSequence(Table2014.P_START_POS),
                                   transitions={'gogogo':'Opening','problem':'end'})
            
            smach.StateMachine.add('Opening', 
                                   Tanguy_Opening.Opening(),
                                   transitions={'endOpening':'MiddleGame', 'motionBlocked':'Unblock',
                                                'askSelector':'MiddleGame', 'nearlyEndMatch':'EndGame'})
            
            smach.StateMachine.add('MiddleGame', 
                                   Strat_MiddleGame.MiddleGame(ActionSelector2014()),
                                   transitions={'endMiddleGame':'EndGame', 'motionBlocked':'Unblock'})
            
            smach.StateMachine.add('EndGame', 
                                   Tanguy_EndGame.EndGame(),
                                   transitions={'endEndGame':'Uninitialisation'})
            
            smach.StateMachine.add('Uninitialisation', 
                                   Strat_Uninitialisation.Uninitialisation(),
                                   transitions={'endUninitialisation':'end'})
            
            smach.StateMachine.add('Unblock', 
                                   Unblock(),
                                   transitions={'succeeded':'MiddleGame', 'nearlyEndMatch':'EndGame'})

   
########################## EXECUTABLE 
#shall be always at the end ! so that every function is defined before
# this main function is the one called by ros
if __name__ == '__main__':
    try:
        StratNode_Tanguy()
    except rospy.ROSInterruptException: 
        rospy.loginfo("handling rospy.ROSInterruptException ...")
        rospy.loginfo("Exiting")
        pass
