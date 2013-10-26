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

from math import *

from arp_master.strat.util.Table2011 import *
from arp_master.strat.util.UtilARD import *

class GetPionBord(PreemptiveStateMachine):
    def __init__(self):
        PreemptiveStateMachine.__init__(self,outcomes=['got','obstacle','endmatch','problem'])
        with self:
            #preemptive states
                       
            PreemptiveStateMachine.add('WaitBecauseObstacle',
                      WaiterState(1.0),
                      transitions={'done':'obstacle'})
            
            PreemptiveStateMachine.add('WaitBecauseRearObstacle',
                      WaiterState(1.0),
                      transitions={'done':'obstacle'})
            
            PreemptiveStateMachine.addPreemptive('EndMatchPreemption',
                                             EndMatchPreempter(-5.0),
                                             transitions={'endMatch':'endmatch'})
            
            PreemptiveStateMachine.add('GotoFacePion1',
                      GotoFacePion1(),
                      transitions={'succeeded':'GotoFacePion2', 'aborted':'TakeMiddle'})
            
            self.setInitialState('GotoFacePion1')
            
            PreemptiveStateMachine.add('GotoFacePion2',
                      GotoFacePion2(),
                      transitions={'succeeded':'GotoFacePion3', 'aborted':'GotoFacePion3'})
            
            PreemptiveStateMachine.add('GotoFacePion3',
                      GotoFacePion3(),
                      transitions={'succeeded':'HandleFirstRedPion', 'aborted':'Reverse1'})
            
            PreemptiveStateMachine.add('HandleFirstRedPion',
                      HandleFirstRedPion(),
                      transitions={'forwardabit':'PreTurn', 'noforward':'Turn'})
            
            PreemptiveStateMachine.add('PreTurn',
                      PreTurn(),
                      transitions={'succeeded':'ForwardABit', 'aborted':'Reverse1'})
                        
            PreemptiveStateMachine.add('ForwardABit',
                      ForwardABit(),
                      transitions={'succeeded':'Turn', 'aborted':'Reverse1'})
            
            PreemptiveStateMachine.add('Turn',
                      Turn(),
                      transitions={'succeeded':'Avance', 'aborted':'Reverse1'})

            PreemptiveStateMachine.add('Avance',
                      Avance(),
                      transitions={'succeeded':'got', 'aborted':'Reverse1'})
            
            ## si on a un probleme on fait un safety drop
            PreemptiveStateMachine.add('Reverse1',
                      Reverse1(),
                      transitions={'succeeded':'TakeMiddle', 'aborted':'problem'})
            
            PreemptiveStateMachine.add('TakeMiddle',
                      TakeMiddle(),
                      transitions={'succeeded':'got', 'aborted':'problem'})
         

class GotoFacePion1(CyclicActionState):
    def createAction(self):
        if Data.pionBordObjectif==None:
            Data.pionBordObjectif=PionBord(0,Data.color)
        cap=AmbiCapRed(-pi,Data.color)
  
        if Data.color=='red':
            xtarget = -0.810
            yoffset = -0.015
        else:
            xtarget = 0.810
            yoffset = -0.010
        cap=AmbiCapRed(-pi,Data.color)
        (xRobot,yRobot)=Data.pionBordObjectif.coord_WhenPionMilieu(cap.angle)
        #je recule de 350, je me decale lateralement
        self.pointcap(xtarget,yRobot+yoffset,cap.angle)     
    
        
class GotoFacePion2(CyclicActionState):
    def createAction(self):
        if Data.pionBordObjectif==None:
            Data.pionBordObjectif=PionBord(0,Data.color)
        cap=AmbiCapRed(-pi,Data.color)
        if Data.color=='red':
            xtarget = -1.190
            yoffset = 0
        else:
            xtarget = 1.190
            yoffset = -0.010
        cap=AmbiCapRed(-pi,Data.color)
        (xRobot,yRobot)=Data.pionBordObjectif.coord_WhenPionMilieu(cap.angle)
        #je vais a l'objectif, je me decale lateralement
        self.pointcap(xtarget,yRobot+yoffset,cap.angle)   

class GotoFacePion3(CyclicActionState):
    def createAction(self):
        self.backward(0.010)

#gestion du cas rouge en haut ou il ne faut pas avancer     
#ET handle last blue    
class HandleFirstRedPion(CyclicState):
    def __init__(self):
        CyclicState.__init__(self, outcomes=['noforward','forwardabit'])
    
    def executeTransitions(self):
        
        if (Data.pionBordObjectif.rang==0 and Data.color=='red') or (Data.pionBordObjectif.rang==3 and Data.color=='blue') :
            return 'noforward'
        else:
            return 'forwardabit'
        
class PreTurn(CyclicActionState):
    def createAction(self):
        if Data.color=='red':
            self.cap(2*pi/3)
        else:
            self.cap(-pi/3)

class ForwardABit(CyclicActionState):
    def createAction(self):
        self.forward(0.100)
        
class Turn(CyclicActionState):
    def createAction(self):
        if Data.color=='red':
            self.cap(radians(50))
        else:
            self.cap(radians(-130))
 
class Avance(CyclicActionState):
    def createAction(self):
        self.forward(0.200) 
 
 ##### pour le blocage: reverse et depose du pion avant de reessayer
class Reverse1(CyclicActionState):
    def createAction(self):
        order=Data.listRewindOrders.pop()#retourne le dernier element et l'enleve de la liste
        if order==None:
            self.dropOnCase(Case(0,0))
        self.executeRewindOrder(order)  
 
class TakeMiddle(CyclicActionState):
    def createAction(self):
        self.dropOnCase(Case(0,0))
 
