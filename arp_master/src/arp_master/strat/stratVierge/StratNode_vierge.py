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

#import the main state machines substates     
from arp_master.strat.stratVierge.Strat_Initialisation import Strat_Initialisation
from arp_master.strat.stratVierge.Strat_StartSequence import Strat_StartSequence
from arp_master.strat.stratVierge.Strat_Opening import Strat_Opening


from arp_master.strat.stratD import Strat_Middlegame_D
from arp_master.strat.stratD import Strat_Endgame_D
from arp_master.strat.sillyWalk import Strat_Uninitialisation

#import utility classes
from arp_master.strat.stratVierge.util.Inputs import Inputs
from arp_master.strat.stratVierge.util.Data import Data
from arp_master.strat.stratVierge.util.TableVierge import *



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
    
        # execute the state machine
        sm.execute()
        

    
    
    
    
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
            smach.StateMachine.add('Middlegame', Strat_Middlegame_D.Middlegame_D(),
                                    transitions={'endMiddlegame':'Endgame'})
            smach.StateMachine.add('Endgame', Strat_Endgame_D.Endgame_D(),
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