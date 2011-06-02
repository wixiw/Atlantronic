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
from arp_master.strat.util.WaiterState import WaiterState
from arp_master.strat.util.Inputs import Inputs
from arp_master.strat.util.Data import Data
from arp_ods.msg import OrderGoal
from arp_ods.msg import OrderAction

from math import *

from arp_master.strat.util.Table2011 import *
from arp_master.strat.util.UtilARD import *

class GetPionBord(PreemptiveStateMachine):
    def __init__(self):
        PreemptiveStateMachine.__init__(self,outcomes=['got','obstacle','endmatch','problem'])
        with self:
            #preemptive states
            PreemptiveStateMachine.addPreemptive('ObstaclePreemption',
                                             ObstaclePreemption(),
                                             transitions={'obstaclepreemption':'WaitBecauseObstacle'})
            
            PreemptiveStateMachine.add('WaitBecauseObstacle',
                      WaiterState(1.0),
                      transitions={'done':'obstacle'})
            
            PreemptiveStateMachine.addPreemptive('RearObstaclePreemption',
                                             RearObstaclePreemption(),
                                             transitions={'rearobstaclepreemption':'WaitBecauseRearObstacle'})
            PreemptiveStateMachine.add('WaitBecauseRearObstacle',
                      WaiterState(1.0),
                      transitions={'done':'obstacle'})
            
            PreemptiveStateMachine.addPreemptive('EndMatchPreemption',
                                             EndMatchPreemption(),
                                             transitions={'endPreemption':'endmatch'})
            
            PreemptiveStateMachine.add('GotoFacePion1',
                      GotoFacePion1(),
                      transitions={'succeeded':'GotoFacePion2', 'aborted':'obstacle'})
            
            self.setInitialState('GotoFacePion1')
            
            PreemptiveStateMachine.add('GotoFacePion2',
                      GotoFacePion2(),
                      transitions={'succeeded':'GotoFacePion3', 'aborted':'GotoFacePion3'})
            
            PreemptiveStateMachine.add('GotoFacePion3',
                      GotoFacePion3(),
                      transitions={'succeeded':'Turn', 'aborted':'obstacle'})
            
            PreemptiveStateMachine.add('Turn',
                      Turn(),
                      transitions={'succeeded':'Avance', 'aborted':'obstacle'})

            PreemptiveStateMachine.add('Avance',
                      Avance(),
                      transitions={'succeeded':'got', 'aborted':'obstacle'})
            
            ## si on a un probleme on fait un safety drop
            PreemptiveStateMachine.add('Reverse1',
                      Reverse1(),
                      transitions={'succeeded':'SafetyDrop', 'aborted':'obstacle'})
            PreemptiveStateMachine.add('SafetyDrop',
                      SafetyDrop(),
                      transitions={'succeeded':'obstacle', 'aborted':'obstacle'})            

class GotoFacePion1(CyclicActionState):
    def createAction(self):
        if Data.pionBordObjectif==None:
            Data.pionBordObjectif=PionBord(0,Data.color)
        cap=AmbiCapRed(-pi,Data.color)
        (xRobot,yRobot)=Data.pionBordObjectif.coord_WhenPionMilieu(cap.angle)
        #je recule de 350, je me decale lateralement
        self.pointcap(xRobot-0.350*cos(cap.angle),yRobot-0.350*sin(cap.angle)-0.005*cos(cap.angle),cap.angle)     
    
        
class GotoFacePion2(CyclicActionState):
    def createAction(self):
        if Data.pionBordObjectif==None:
            Data.pionBordObjectif=PionBord(0,Data.color)
        cap=AmbiCapRed(-pi,Data.color)
        (xRobot,yRobot)=Data.pionBordObjectif.coord_WhenPionMilieu(cap.angle)
        #je vais a l'objectif, je me decale lateralement
        self.pointcap(xRobot+0.250*cos(cap.angle),yRobot+0.005*cos(cap.angle),cap.angle)   

class GotoFacePion3(CyclicActionState):
    def createAction(self):
        self.backward(0.010)
        
class Turn(CyclicActionState):
    def createAction(self):
        if Data.color=='red':
            self.cap(radians(50))
        else:
            self.cap(radians(-130))
 
class Avance(CyclicActionState):
    def createAction(self):
        self.forward(0.100) 
 
 ##### pour le blocage: reverse et depose du pion avant de reessayer
class Reverse1(CyclicActionState):
    def createAction(self):
        order=Data.listReplayOrders.pop()#retourne le dernier element et l'enleve de la liste
        if order==None:
            self.dropOnCase(Case(0,0))
        self.executeReplayOrder(order)  
        
class SafetyDrop(CyclicActionState):
    def createAction(self):
        self.dropOnCase(AmbiCaseRed(-3,-3,Data.color)) 
        
 
class EndMatchPreemption(PreemptiveCyclicState):
    def __init__(self):
        PreemptiveCyclicState.__init__(self, outcomes=['endPreemption'])
        self.match_duration=rospy.get_param("/match_duration")

    def preemptionCondition(self):
        if (rospy.get_rostime()-Data.start_time).to_sec()>self.match_duration:
            return True
        else:
            return False
       
    def executeTransitions(self):
        return 'endPreemption'
    
    
class ObstaclePreemption(FrontObstaclePreempter):
    def __init__(self):
        FrontObstaclePreempter.__init__(self, outcomes=['obstaclepreemption'])
       
    def executeTransitions(self):
        return 'obstaclepreemption'
    
class RearObstaclePreemption(RearObstaclePreempter):
    def __init__(self):
        RearObstaclePreempter.__init__(self, outcomes=['rearobstaclepreemption'])
       
    def executeTransitions(self):
        return 'rearobstaclepreemption'