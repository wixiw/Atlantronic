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
from arp_master.strat.util.Table2011 import *
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
                                   transitions={'startunplug':'Droite1'})
            
            smach.StateMachine.add('Cap1', Cap1(),
                                   transitions={'succeeded':'Cap2', 'aborted':'end'})
            smach.StateMachine.add('Cap2', Cap2(),
                                   transitions={'succeeded':'Cap3', 'aborted':'end'})
            smach.StateMachine.add('Cap3', Cap3(),
                                   transitions={'succeeded':'Cap4', 'aborted':'end'})
            smach.StateMachine.add('Cap4', Cap4(),
                                   transitions={'succeeded':'Cap5', 'aborted':'end'})
            smach.StateMachine.add('Cap5', Cap5(),
                                   transitions={'succeeded':'Cap6', 'aborted':'end'})
            smach.StateMachine.add('Cap6', Cap6(),
                                   transitions={'succeeded':'Cap7', 'aborted':'end'})
            smach.StateMachine.add('Cap7', Cap7(),
                                   transitions={'succeeded':'Cap1', 'aborted':'end'})
            
            smach.StateMachine.add('Droite1', Droite1(),
                                   transitions={'succeeded':'Droite2', 'aborted':'end'})
            smach.StateMachine.add('Droite2', Droite2(),
                                   transitions={'succeeded':'Droite1', 'aborted':'end'})
            
            
            smach.StateMachine.add('Fantom1', Fantom1(),
                                   transitions={'succeeded':'Fantom2', 'aborted':'end'})
            smach.StateMachine.add('Fantom2', Fantom2(),
                                   transitions={'succeeded':'Fantom3', 'aborted':'end'})
            smach.StateMachine.add('Fantom3', Fantom3(),
                                   transitions={'succeeded':'Fantom4', 'aborted':'end'})
            smach.StateMachine.add('Fantom4', Fantom4(),
                                   transitions={'succeeded':'Init', 'aborted':'end'})
            
            smach.StateMachine.add('Rush1', Rush1(),
                                   transitions={'succeeded':'Rush2', 'aborted':'end'})
            smach.StateMachine.add('Rush2', Rush2(),
                                   transitions={'succeeded':'Rush3', 'aborted':'end'})
            smach.StateMachine.add('Rush3', Rush3(),
                                   transitions={'succeeded':'Rush4', 'aborted':'end'})
            smach.StateMachine.add('Rush4', Rush4(),
                                   transitions={'succeeded':'Rush5', 'aborted':'end'})
            smach.StateMachine.add('Rush5', Rush5(),
                                   transitions={'succeeded':'Rush6', 'aborted':'end'})
            smach.StateMachine.add('Rush6', Rush6(),
                                   transitions={'succeeded':'Rush7', 'aborted':'end'})
            smach.StateMachine.add('Rush7', Rush7(),
                                   transitions={'succeeded':'Rush8', 'aborted':'end'})
            smach.StateMachine.add('Rush8', Rush8(),
                                   transitions={'succeeded':'Rush9', 'aborted':'end'})
            smach.StateMachine.add('Rush9', Rush9(),
                                   transitions={'succeeded':'Init', 'aborted':'end'})

class Rush1(CyclicActionState):
    def createAction(self):
        self.pointcap(-0.850,0.840,-pi/2)
        
class Rush2(CyclicActionState):
    def createAction(self):
        self.pointcap(-0.850,0.273,-pi)
        
class Rush3(CyclicActionState):
    def createAction(self):
        self.pointcap(-1.180,0.273,-pi)
        
class Rush4(CyclicActionState): 
    def createAction(self):
        self.cap(-2*pi/3)

class Rush5(CyclicActionState): 
    def createAction(self):
        self.forward(0.100)
        
class Rush6(CyclicActionState): 
    def createAction(self):
        self.cap(-pi/2)
        
class Rush7(CyclicActionState): 
    def createAction(self):
        self.forward(0.050)
        
class Rush8(CyclicActionState): 
    def createAction(self):
        self.cap(0)
        
class Rush9(CyclicActionState): 
    def createAction(self):
        self.forward(0.300)

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
   
class Cap1(CyclicActionState):
    def createAction(self):
        self.cap(0)
        
class Cap2(CyclicActionState):
    def createAction(self):
        self.cap(pi/8)

class Cap3(CyclicActionState):
    def createAction(self):
        self.cap(0)
        
class Cap4(CyclicActionState):
    def createAction(self):
        self.cap(pi/2)

class Cap5(CyclicActionState):
    def createAction(self):
        self.cap(0)
        
class Cap6(CyclicActionState):
    def createAction(self):
        self.cap(7*pi/8)

class Cap7(CyclicActionState):
    def createAction(self):
        self.cap(0)
        
class Droite1(CyclicActionState):
    def createAction(self):
        #self.pointcap(1,0,0)
        self.pointcap(1.00,0.0,0)
        
class Droite2(CyclicActionState):
    def createAction(self):
        self.pointcap_reverse(0,0,0)       

class Fantom1(CyclicActionState):
    def createAction(self):
        self.pointcap(0.8,0.8,pi/2)
        
class Fantom2(CyclicActionState):
    def createAction(self):
        self.pointcap_reverse(0,0,0)    

class Fantom3(CyclicActionState):
    def createAction(self):
        self.pointcap(0,0.4,pi)  
        
class Fantom4(CyclicActionState):
    def createAction(self):
        self.pointcap_reverse(0,0,0)  
        
########################## EXECUTABLE 
#shall be always at the end ! so that every function is defined before
# main function, called by ros
if __name__ == '__main__':
    try:
        MotionTestingStratNode()
    except rospy.ROSInterruptException: pass