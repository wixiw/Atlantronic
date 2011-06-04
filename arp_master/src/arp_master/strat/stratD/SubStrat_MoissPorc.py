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
from random import *

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
                                      transitions={'succeeded':'Moiss2', 'aborted':'RandomDrop'})
            
            
            PreemptiveStateMachine.add('Moiss2',
                                        Moiss2(),
                                        transitions={'succeeded':'Moiss3', 'aborted':'RandomDrop'})
            
            PreemptiveStateMachine.add('Moiss3',
                                        Moiss3(),
                                        transitions={'succeeded':'DropPion', 'aborted':'RandomDrop'})
            
            PreemptiveStateMachine.add('DropPion',
                                        SubStrat_DropPion.DropPion(),
                                        transitions={'dropped':'Pousse1_1','endmatch':'endmatch'})

            PreemptiveStateMachine.add('Pousse1_1',
                                        Pousse1_1(),
                                        transitions={'succeeded':'Pousse1_2', 'aborted':'DropWalk1_1'})

            PreemptiveStateMachine.add('Pousse1_2',
                                        Pousse1_2(),
                                        transitions={'succeeded':'Pousse1_3', 'aborted':'Pousse1_3'})

            PreemptiveStateMachine.add('Pousse1_3',
                                        Pousse1_3(),
                                        transitions={'succeeded':'DropWalk1_1', 'aborted':'Pousse1_3'})
       
            
            PreemptiveStateMachine.add('DropWalk2_1',
                                        DropWalk2_1(),
                                        transitions={'succeeded':'DropWalk2_2', 'aborted':'DropWalk1_2'})
            PreemptiveStateMachine.add('DropWalk2_2',
                                        DropTurn(),
                                        transitions={'succeeded':'DropWalk2_3', 'aborted':'DropWalk2_3'})
            PreemptiveStateMachine.add('DropWalk2_3',
                                        DropBack(),
                                        transitions={'succeeded':'DropWalk1_1', 'aborted':'RandomDrop'})
            

            PreemptiveStateMachine.add('DropWalk1_1',
                                        DropWalk1_1(),
                                        transitions={'succeeded':'DropWalk1_2', 'aborted':'DropWalk2_2'})
            PreemptiveStateMachine.add('DropWalk1_2',
                                        DropTurn(),
                                        transitions={'succeeded':'DropWalk1_3', 'aborted':'DropWalk1_3'})
            PreemptiveStateMachine.add('DropWalk1_3',
                                        DropBack(),
                                        transitions={'succeeded':'DropWalk2_1', 'aborted':'RandomDrop'})
          
          
            PreemptiveStateMachine.add('RandomDrop',
                                        RandomDrop(),
                                        transitions={'succeeded':'RandomDropTurn', 'aborted':'RandomDrop'})  
            PreemptiveStateMachine.add('RandomDropTurn',
                                        DropTurn(),
                                        transitions={'succeeded':'RandomDropBack', 'aborted':'RandomDropBack'})
            PreemptiveStateMachine.add('RandomDropBack',
                                        DropBack(),
                                        transitions={'succeeded':'RandomDrop', 'aborted':'RandomDrop'})
            
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


class DropWalk1_1(CyclicActionState):
    def createAction(self):
        self.dropOnCase(AmbiCaseRed(1,5,Data.color))


class DropWalk2_1(CyclicActionState):
    def createAction(self):
        self.dropOnCase(AmbiCaseRed(1,-3,Data.color))

    
#poussage au fond
class Pousse1_1(CyclicActionState):
    def createAction(self):
        pose=AmbiPoseRed(0,0.125,0,Data.color)
        self.pointcap_reverse(pose.x,pose.y,pose.theta)  

class Pousse1_2(CyclicActionState):
    def createAction(self):
        self.dropOnCase(AmbiCaseRed(5,1,Data.color))
        
class Pousse1_3(CyclicActionState):
    def createAction(self):
        pose=AmbiPoseRed(0,0.125,0,Data.color)
        self.pointcap_reverse(pose.x,pose.y,pose.theta)
        
        
        
class RandomDrop(CyclicActionState):
    def createAction(self):
        dropCase=Data.randomDropList[randint(0,len(Data.randomDropList)-1)]
        self.dropOnCase(dropCase.getCase(Data.color))
        
class ReturnCenter(CyclicActionState):
    def createAction(self):
        pose=AmbiPoseRed(-0.125,0,Inputs.gettheta(),Data.color)
        self.pointcap_reverse(pose.x,pose.y,pose.theta)
        


  
