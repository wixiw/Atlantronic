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
from CyclicState import CyclicState
from CyclicActionState import CyclicActionState

from Inputs import Inputs
from Data import Data
from Table2011 import *
from math import pi

###########################  TEMPORAL BEHAVIOR

class MotionTestingStratNode():
    
    def __init__(self):
        
        #creation of the node
        rospy.init_node('MotionTestingStratNode')
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
        rospy.loginfo("Welcome to the Motion tester.")
        rospy.loginfo("I will execute sequence when start is unplugged.")
        rospy.loginfo("******************************************************")
    
        sm.execute()

    
############################## STATE MACHINE
class MainStateMachine(smach.StateMachine):
    def __init__(self):
        smach.StateMachine.__init__(self,outcomes=['end'])
        with self:
            smach.StateMachine.add('Init', Init(),
                                   transitions={'initstateok':'WaitForStartPlug'})
            smach.StateMachine.add('WaitForStartPlug', WaitForStartPlug(),
                                   transitions={'startplug':'WaitForStartUnplug'})
            smach.StateMachine.add('WaitForStartUnplug', WaitForStartUnplug(),
                                   transitions={'startunplug':'Step1'})
            smach.StateMachine.add('Step1', Step1(),
                                   transitions={'succeeded':'Step2', 'aborted':'end'})
            smach.StateMachine.add('Step2', Step2(),
                                   transitions={'succeeded':'Step3', 'aborted':'end'})
            smach.StateMachine.add('Step3', Step3(),
                                   transitions={'succeeded':'end', 'aborted':'end'})

class Init(CyclicState):
    def __init__(self):
        CyclicState.__init__(self, outcomes=['initstateok'])

    def executeTransitions(self):
       # the only condition verified to go on is that the start is not put
       if Inputs.getstart()==1:
           return 'initstateok'
        
class WaitForStartPlug(CyclicState):
    def __init__(self):
        CyclicState.__init__(self, outcomes=['startplug'])
    
    def executeTransitions(self):
       if Inputs.getstart()==0:
            return 'startplug'
        
class WaitForStartUnplug(CyclicState):
    def __init__(self):
        CyclicState.__init__(self, outcomes=['startunplug'])
    
    def executeTransitions(self):
       if Inputs.getstart()==1:
            return 'startunplug'
        
    def executeOut(self):
        self.setPosition(0,0,0)
        self.enableDrive()
   
class Step1(CyclicActionState):
    def createAction(self):
        self.cap(-pi/2)
        
class Step2(CyclicActionState):
    def createAction(self):
        self.pointcap_reverse(0,0,0)

class Step3(CyclicActionState):
    def createAction(self):
        self.pointcap(1,0,0)
        
########################## EXECUTABLE 
#shall be always at the end ! so that every function is defined before
# main function, called by ros
if __name__ == '__main__':
    try:
        MotionTestingStratNode()
    except rospy.ROSInterruptException: pass