#!/usr/bin/env python

#libraries for ROS
import roslib; roslib.load_manifest('arp_master')

from arp_master import *
from std_msgs.msg import Bool
from ActuatorStates import *

###########################  TEMPORAL BEHAVIOR

class ManualStratNode():
    
    def __init__(self):
        
        #creation of the node
        rospy.init_node('ManualStratNode')
        # recuperation des parametres)
        #creation of the cadencer for all states
        Data.stateMachineRate =rospy.Rate(10)
        #linking of the input in Inputs to the topics
        Inputs.link()
        #creation of statemachine
        sm=MainStateMachine()
    
        #welcome message
        rospy.loginfo("******************************************************")
        rospy.loginfo("Welcome to the Manual monitoring strat.")
        rospy.loginfo("I will execute sequence when start is unplugged.")
        rospy.loginfo("******************************************************")
        
        # initialise the smach introspection server to view the state machine with :
        #  rosrun smach_viewer smach_viewer.py
        sis = smach_ros.IntrospectionServer('strat_server', sm, '/ManualStratNode')
        sis.start()
        sm.execute()
        sis.stop()

    
############################## STATE MACHINE
class MainStateMachine(smach.StateMachine):
    def __init__(self):
        smach.StateMachine.__init__(self,outcomes=['end'])
        with self:
            smach.StateMachine.add('Initialisation', Strat_Initialisation.Initialisation(),
                                   transitions={'endInitialisation':'StartSequence', 'failed':'end'})
            smach.StateMachine.add('StartSequence', Strat_StartSequence.StartSequence(0,0,0),
                                   transitions={'gogogo':'Run','problem':'end'}) 
            smach.StateMachine.add('Run', Run(),
                                   transitions={'exit':'end'}) 

     
class Go(CyclicState):
    def __init__(self):
        CyclicState.__init__(self,outcomes=['exit'])
        self.pub = rospy.Publisher('/Strat/go', Bool)
        
    def executeIn(self):
        self.pub.publish(Bool(True))
        
    def executeTransitions(self):
        return
    
class StateSleep(CyclicState):
    def __init__(self):
        CyclicState.__init__(self,outcomes=[])


#La machine a etat Run correspond aux 6 boutons 1(A),2(B),3(C),4(D),9(start),10(select) de la manette type PS2.
class Run(smach.StateMachine):
    def __init__(self):
        smach.StateMachine.__init__(self,outcomes=['exit'])
        with self:
            smach.StateMachine.add('State1', 
                                   State1(),
                                   transitions={'2':'State2', 
                                                '3':'State3',
                                                '4':'State4',
                                                '9':'State9',
                                                '10':'State10',
                                                'timeout':'StateSleep'})
            smach.StateMachine.add('State2', 
                                   State2(),
                                   transitions={'1':'State1', 
                                                '3':'State3',
                                                '4':'State4',
                                                '9':'State9',
                                                '10':'State10',
                                                'timeout':'StateSleep'})
            smach.StateMachine.add('State3', 
                                   State3(),
                                   transitions={'1':'State1', 
                                                '2':'State2',
                                                '4':'State4',
                                                '9':'State9',
                                                '10':'State10',
                                                'timeout':'StateSleep'})
            smach.StateMachine.add('State4', 
                                   State4(),
                                   transitions={'1':'State1', 
                                                '2':'State2',
                                                '3':'State3',
                                                '9':'State9',
                                                '10':'State10',
                                                'timeout':'StateSleep'})
            smach.StateMachine.add('State9', 
                                   State9(),
                                   transitions={'1':'State1', 
                                                '2':'State2',
                                                '3':'State3',
                                                '4':'State4',
                                                '10':'State10',
                                                'timeout':'StateSleep'})
            smach.StateMachine.add('State10', 
                                   State10(),
                                   transitions={'1':'State1', 
                                                '2':'State2',
                                                '3':'State3',
                                                '4':'State4',
                                                '9':'State9',
                                                'timeout':'StateSleep'})
            
            smach.StateMachine.add('StateSleep', 
                                   StateSleep(),
                                   transitions={'timeout':'StateSleep'})
      
        
########################## EXECUTABLE 
#shall be always at the end ! so that every function is defined before
# main function, called by ros
if __name__ == '__main__':
    try:
        ManualStratNode()
    except smach.InvalidTransitionError:
        rospy.loginfo("handling smach.InvalidTransitionError ...")
        rospy.loginfo("Exiting")
        pass
    
    except rospy.ROSInterruptException: 
        rospy.loginfo("handling rospy.ROSInterruptException ...")
        rospy.loginfo("Exiting")
        pass