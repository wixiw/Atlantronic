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
from arp_master.strat.util.CyclicState import CyclicState
from arp_master.strat.util.CyclicActionState import CyclicActionState

from arp_master.strat.util.Inputs import Inputs
from arp_master.strat.util.Data import Data

from math import *

###########################  TEMPORAL BEHAVIOR

class MotionTestingStratNode():
    
    def __init__(self):
        
        #creation of the node
        rospy.init_node('MotionTestingStratNode')
        # recuperation des parametres)
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
                                   transitions={'startunplug':'Move1'})
            
            
            smach.StateMachine.add('Move1', Move1(),
                                   transitions={'succeeded':'Move2', 'aborted':'end'})
            smach.StateMachine.add('Move2', Move2(),
                                   transitions={'succeeded':'Move3', 'aborted':'end'})
            smach.StateMachine.add('Move3', Move3(),
                                   transitions={'succeeded':'Move4', 'aborted':'end'})
            smach.StateMachine.add('Move4', Move4(),
                                   transitions={'succeeded':'Move1', 'aborted':'end'})            


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
        self.enablePower()
   

        
class Move1(CyclicActionState):
    def createAction(self):
        self.omnidirect(0.5,0.5,pi/2)
        
class Move2(CyclicActionState):
    def createAction(self):
        self.omnidirect(-0.5,0.5,-pi/2)       

class Move3(CyclicActionState):
    def createAction(self):
        self.omnidirect_cpoint(0.3,0,0,0,-0.4,0)   

class Move4(CyclicActionState):
    def createAction(self):
        self.omnidirect_cpoint(0.3,0,0,0,-0.4,pi/2)   
        
########################## EXECUTABLE 
#shall be always at the end ! so that every function is defined before
# main function, called by ros
if __name__ == '__main__':
    try:
        MotionTestingStratNode()
    except rospy.ROSInterruptException: pass