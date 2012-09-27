#!/usr/bin/env python

#libraries for ROS
import roslib; roslib.load_manifest('arp_master')

from arp_master import *

#import the main state machines substates     
#from a0_initialisation import Strat_Initialisation => utilisation de l'etat commun
#from a1_startSequence import Strat_StartSequence => utilisation de l'etat commun
from a2_opening import Strat_Opening
from a3_middleGame import Strat_MiddleGame
from a4_endGame import Strat_EndGame
#from a5_uninitialisation import Strat_Uninitialisation => utilisation de l'etat commun

from arp_master.strat_2013 import *

###########################  TEMPORAL BEHAVIOR

class StratNode_vierge():
    
    def __init__(self):
        
        #creation of the node
        rospy.init_node('StratNode')
        #recuperation des parametres (on a besoin d'etre un noeud pour ca
        Table2013.getParams()
        Robot2013.getParams()
        #creation of the cadencer for all states
        Data.stateMachineRate =rospy.Rate(10)
        #linking of the input in Inputs to the topics
        Inputs.link()
        #creation of statemachine
        sm=MainStateMachine()
    
        #welcome message
        rospy.loginfo("******************************************************")
        rospy.loginfo("Welcome to Advanced Robotics Platform. I am StratNode 2013.")
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
                                   transitions={'endInitialisation':'StartSequence','failed':'end'})
            smach.StateMachine.add('StartSequence', Strat_StartSequence.StartSequence(1.140 + Robot2013.CDG_POSE.x,0.75,-pi),
                                   transitions={'gogogo':'Opening','problem':'end'})
            smach.StateMachine.add('Opening', Strat_Opening.Opening(),
                                    transitions={'endOpening':'MiddleGame','problem':'MiddleGame'})
            smach.StateMachine.add('MiddleGame', Strat_MiddleGame.MiddleGame(),
                                    transitions={'endMiddleGame':'EndGame'})
            smach.StateMachine.add('EndGame', Strat_EndGame.EndGame(),
                                    transitions={'endEndGame':'Uninitialisation'})
            smach.StateMachine.add('Uninitialisation', Strat_Uninitialisation.Uninitialisation(),
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