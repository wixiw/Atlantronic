#!/usr/bin/env python

import roslib; roslib.load_manifest('arp_master')
import rospy
from rospy import loginfo
import smach
import smach_ros
import smach_msgs

from random import *

from arp_master.strat.util.CyclicState import CyclicState
from arp_master.strat.util.CyclicActionState import CyclicActionState
from arp_master.strat.util.PreemptiveStateMachine import PreemptiveStateMachine
from arp_master.strat.util.PreemptiveCyclicState import PreemptiveCyclicState
from arp_master.strat.util.ObstaclePreempter import FrontObstaclePreempter
from arp_master.strat.util.ObstaclePreempter import RearObstaclePreempter
from arp_master.strat.util.EndMatchPreempter import EndMatchPreempter
from arp_master.strat.util.WaiterState import WaiterState
from arp_master.strat.util.Inputs import Inputs
from arp_master.strat.util.Data import Data
from arp_ods.msg import OrderGoal
from arp_ods.msg import OrderAction
import SubStrat_DropPion

from math import *

from arp_master.strat.util.Table2011 import *
from arp_master.strat.util.UtilARD import *

class MoissPorc(PreemptiveStateMachine):
    def __init__(self):
        PreemptiveStateMachine.__init__(self,outcomes=['endmatch'])
        with self:
            PreemptiveStateMachine.addPreemptive('EndMatchPreemption',
                                             EndMatchPreempter(-5.0),
                                             transitions={'endMatch':'endmatch'})
            PreemptiveStateMachine.add('InitMoiss',
                                        InitMoiss(),
                                        transitions={'done':'Moiss1'})
            self.setInitialState('InitMoiss')
            
            PreemptiveStateMachine.add('Moiss1',
                                      Moiss1(),
                                      transitions={'succeeded':'Moiss2', 'aborted':'endmatch'})
            #as initial state is not the preemptive one, it is necessary to add the information here !
            
            
            PreemptiveStateMachine.add('Moiss2',
                                        Moiss2(),
                                        transitions={'succeeded':'Moiss3', 'aborted':'endmatch'})
            
            PreemptiveStateMachine.add('Moiss3',
                                        Moiss3(),
                                        transitions={'succeeded':'DropPion', 'aborted':'endmatch'})
            
            PreemptiveStateMachine.add('DropPion',
                                        SubStrat_DropPion.DropPion(),
                                        transitions={'dropped':'Walk1_1','endmatch':'endmatch'})

            PreemptiveStateMachine.add('Walk1_1',
                                        Walk1_1(),
                                        transitions={'succeeded':'DropWalk1_1', 'aborted':'endmatch'})
            PreemptiveStateMachine.add('DropWalk1_1',
                                        DropWalk1_1(),
                                        transitions={'succeeded':'DropWalk1_2', 'aborted':'endmatch'})
            PreemptiveStateMachine.add('DropWalk1_2',
                                        DropTurn(),
                                        transitions={'succeeded':'DropWalk1_3', 'aborted':'endmatch'})
            PreemptiveStateMachine.add('DropWalk1_3',
                                        DropBack(),
                                        transitions={'succeeded':'Walk2_1', 'aborted':'endmatch'})
            
            PreemptiveStateMachine.add('Walk2_1',
                                        Walk2_1(),
                                        transitions={'succeeded':'DropWalk2_1', 'aborted':'endmatch'})
            PreemptiveStateMachine.add('DropWalk2_1',
                                        DropWalk2_1(),
                                        transitions={'succeeded':'DropWalk2_2', 'aborted':'endmatch'})
            PreemptiveStateMachine.add('DropWalk2_2',
                                        DropTurn(),
                                        transitions={'succeeded':'DropWalk2_3', 'aborted':'endmatch'})
            PreemptiveStateMachine.add('DropWalk2_3',
                                        DropBack(),
                                        transitions={'succeeded':'Bidon1', 'aborted':'endmatch'})
            
            PreemptiveStateMachine.add('Bidon1',
                                        Bidon1(),
                                        transitions={'succeeded':'Bidon2', 'aborted':'endmatch'})
            PreemptiveStateMachine.add('Bidon2',
                                        Bidon2(),
                                        transitions={'succeeded':'Bidon1', 'aborted':'endmatch'})
            
class InitMoiss(CyclicState):
    def __init__(self):
        CyclicState.__init__(self, outcomes=['done'])
    
    def executeTransitions(self):
        return 'done'
        

class Moiss1(CyclicActionState):
    def createAction(self):
        self.dropOnCase(AmbiCaseRed(-2,4,Data.color))
        
class Moiss2(CyclicActionState):
    def createAction(self):
        self.dropOnCase(AmbiCaseRed(-2,2,Data.color))
        
class Moiss3(CyclicActionState):
    def createAction(self):
        self.dropOnCase(AmbiCaseRed(-2,-2,Data.color))
        
class DropMoiss1(CyclicActionState):
    def createAction(self):
        self.dropOnCase(AmbiCaseRed(3,-1,Data.color))

##### CES ETATS LA SONT TOUT LE TEMPS UTILISES
class DropTurn(CyclicActionState):
    def createAction(self):
        self.cap(Inputs.gettheta()+radians(15))

class DropBack(CyclicActionState):
    def createAction(self):
        self.backward(0.35)

#### WALK

class Walk1_1(CyclicActionState):
    def createAction(self):
        self.dropOnCase(AmbiCaseRed(-3,3,Data.color))  

class DropWalk1_1(CyclicActionState):
    def createAction(self):
        self.dropOnCase(AmbiCaseRed(-5,3,Data.color))

class Walk2_1(CyclicActionState):
    def createAction(self):
        self.dropOnCase(AmbiCaseRed(0,0,Data.color))  

class DropWalk2_1(CyclicActionState):
    def createAction(self):
        self.dropOnCase(AmbiCaseRed(1,-3,Data.color))

class Bidon1(CyclicActionState):
    def createAction(self):
        self.cap(0) 
class Bidon2(CyclicActionState):
    def createAction(self):
        self.cap(pi/2)     
        
        
  
