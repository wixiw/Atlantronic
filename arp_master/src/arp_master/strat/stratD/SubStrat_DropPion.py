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

from math import pi

from arp_master.strat.util.Table2011 import *
from arp_master.strat.util.UtilARD import *

class DropPion(PreemptiveStateMachine):
    def __init__(self):
        PreemptiveStateMachine.__init__(self,outcomes=['dropped','endmatch'])
        with self:
            #preemptive states
            
            PreemptiveStateMachine.addPreemptive('EndMatchPreemption',
                                             EndMatchPreempter(-5.0),
                                             transitions={'endMatch':'endmatch'})
            
            PreemptiveStateMachine.add('Drop1',
                      Drop1(),
                      transitions={'succeeded':'Drop11', 'aborted':'WaitIfPbDrop1'})
            
            self.setInitialState('Drop1')
            
            PreemptiveStateMachine.add('WaitIfPbDrop1',
                      WaiterState(1.0),
                      transitions={'done':'Drop1'})
            
            PreemptiveStateMachine.add('Drop11',
                      Drop11(),
                      transitions={'succeeded':'Drop2', 'aborted':'dropped'})
            
            PreemptiveStateMachine.add('Drop2',
                      Drop2(),
                      transitions={'succeeded':'dropped', 'aborted':'dropped'})
            


        

class Drop1(CyclicActionState): 
    def __init__(self):
        CyclicActionState.__init__(self)


    def createAction(self):
        #je parcours toutes les cases et je prend la premiere non occupee
        for dropcase in Data.casePriority:
            if dropcase.status=='VIRGIN':
                #je cree la bonne case bleue ou rouge, je stocke dans Data pour les autres etats
                Data.currentDropCase=dropcase
                #je vais dessus
                self.dropOnCase(dropcase.getCase(Data.color))
                rospy.loginfo("DROP >> selectionne le VIRGIN :"+str(dropcase.getCase(Data.color)))
                return
            
        for dropcase in Data.casePriority:
            if dropcase.status=='BLOCKED':
                #je cree la bonne case bleue ou rouge, je stocke dans Data pour les autres etats
                Data.currentDropCase=dropcase
                #je vais dessus
                self.dropOnCase(dropcase.getCase(Data.color))
                rospy.loginfo("DROP >> selectionne le BLOCKED :"+str(dropcase.getCase(Data.color)))
                return
            
        #si j'ai fini la liste, alors je prend la premiere (devrait pas arriver)
        self.dropOnCase(self.casePriority[0].getCase(Data.color))
        rospy.loginfo("DROP >> a rien trouve :"+str(dropcase.getCase(Data.color)))
            
    def executeOut(self):
        #si j'ai reussi a terminer mon ordre je note que la case est occupee
        if self.trans=='succeeded':
            Data.currentDropCase.status='DONE'
        if self.trans=='aborted':
            Data.currentDropCase.status='BLOCKED'
      
      
#tourne un peu pour degager le nez 
class Drop11(CyclicActionState):
    def createAction(self):
        self.cap(Inputs.gettheta()+radians(15))
        
class Drop2(CyclicActionState):
    def createAction(self):
        self.backward(0.35)    
    
