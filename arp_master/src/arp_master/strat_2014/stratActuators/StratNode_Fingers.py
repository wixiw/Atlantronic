#!/usr/bin/env python

#libraries for ROS
import roslib; roslib.load_manifest('arp_master')

from arp_master import *

from arp_master.util import *
from arp_master.fsmFramework import *
from arp_master.commonStates import *
from arp_master.strat_2014 import *
from arp_master.actuators import *

###########################  TEMPORAL BEHAVIOR

class StratNode_vierge():
    
    def __init__(self):
        
        #creation of the node
        rospy.init_node('StratNode_Finger')
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
        rospy.loginfo("Welcome to Advanced Robotics Platform. I am StratNode Finger.")
        rospy.loginfo("******************************************************")
    
        # initialise the smach introspection server to view the state machine with :
        #  rosrun smach_viewer smach_viewer.py
        sis = smach_ros.IntrospectionServer('strat_server', sm, '/StratNode_Finger')
        sis.start()
        sm.execute()
        
        sis.stop()
    
############################## MAIN STATE MACHINE CREATION
class MainStateMachine(smach.StateMachine):
    def __init__(self):
        smach.StateMachine.__init__(self, outcomes=['end'])
        with self:
            smach.StateMachine.add('Initialisation', 
                                   InitStates2014.InitSequence2014(),
                                   transitions={'endInitialisation':'PrepareActuators','failed':'Uninitialisation'})
             
            smach.StateMachine.add('Uninitialisation', 
                                   Strat_Uninitialisation.Uninitialisation(),
                                   transitions={'endUninitialisation':'end'})
            
            #Bootup actuators, show their aliveness, and go to default position. All actuators are driven
            smach.StateMachine.add('PrepareActuators',
                       PrepareActuators(),
                       transitions={'prepared':'WaitABit','problem':'Uninitialisation'})

            smach.StateMachine.add('WaitABit',
                       WaiterState(1),
                       transitions={'timeout':'AmbiShootOneBallLeft'})
     
            smach.StateMachine.add('AmbiPickLeft',
                       AmbiFingerPickObject('Left'),
                       transitions={'shot':'AmbiPickRight', 'blocked':'Uninitialisation'})
             
            smach.StateMachine.add('AmbiPickRight',
                       AmbiFingerPickObject('Right'),
                       transitions={'shot':'WaitABit', 'blocked':'Uninitialisation'})
             

   
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