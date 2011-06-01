#!/usr/bin/env python

import roslib; roslib.load_manifest('arp_master')
import rospy
import smach
import smach_ros
import smach_msgs

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

class Opening_D(PreemptiveStateMachine):
    def __init__(self):
        PreemptiveStateMachine.__init__(self, outcomes=['endOpening', 'problem'])
        with self:
            PreemptiveStateMachine.addPreemptive('ObstaclePreemption',
                      ObstaclePreemption(),
                      transitions={'avoidByMilieu':'WaitBeforeMilieu','jump':'WaitBeforeJump'})
            
            PreemptiveStateMachine.add('WaitBeforeMilieu',
                      WaiterState(3.0),
                      transitions={'done':'Milieu1'})
            
            PreemptiveStateMachine.add('WaitBeforeJump',
                      WaiterState(3.0),
                      transitions={'done':'endOpening'})
            
            PreemptiveStateMachine.addPreemptive('RearObstaclePreemption',
                      RearObstaclePreemption(),
                      transitions={'rearobstaclepreemption':'WaitBecauseRearObstacle'})
            
            PreemptiveStateMachine.add('WaitBecauseRearObstacle',
                      WaiterState(3.0),
                      transitions={'done':'endOpening'})
            
            
            PreemptiveStateMachine.add('EscapeStartpoint',
                      EscapeStartpoint(),
                      transitions={'succeeded':'LigneVert1', 'aborted':'Milieu1'})
            
            self.setInitialState('EscapeStartpoint')
            
            PreemptiveStateMachine.add('LigneVert1',
                      LigneVert1(),
                      transitions={'succeeded':'LigneVert2', 'aborted':'Milieu1'})

            PreemptiveStateMachine.add('LigneVert2',
                      LigneVert2(),
                      transitions={'succeeded':'LigneVert3', 'aborted':'Milieu1'})

            PreemptiveStateMachine.add('LigneVert3',
                      LigneVert3(),
                      transitions={'succeeded':'SwitchRedBlue', 'aborted':'Milieu1'})      
                  
            # attention il y a deux branches, suivant si on est rouge ou bleu
            #rouge
            PreemptiveStateMachine.add('SwitchRedBlue',
                      SwitchRedBlue(),
                      transitions={'red':'DropRed1', 'blue':'DropBlue1'})  
            
            
            PreemptiveStateMachine.add('DropRed1',
                      DropRed1(),
                      transitions={'succeeded':'RecalRed', 'aborted':'Milieu1'})   
            
            PreemptiveStateMachine.add('RecalRed',
                      RecalRed(),
                      transitions={'done':'DropRed2'})   
            
            PreemptiveStateMachine.add('DropRed2',
                      DropRed2(),
                      transitions={'succeeded':'DropRed3', 'aborted':'Milieu1'})   
 
            PreemptiveStateMachine.add('DropRed3',
                      DropRed3(),
                      transitions={'succeeded':'endOpening', 'aborted':'Milieu1'})  
 
            #bleu 
            PreemptiveStateMachine.add('DropBlue1',
                      DropBlue1(),
                      transitions={'succeeded':'DropBlue2', 'aborted':'Milieu1'})   
            
            PreemptiveStateMachine.add('DropBlue2',
                      DropBlue2(),
                      transitions={'succeeded':'DropBlue3', 'aborted':'Milieu1'}) 
            
            PreemptiveStateMachine.add('DropBlue3',
                      DropBlue3(),
                      transitions={'succeeded':'DropBlue4', 'aborted':'Milieu1'}) 
                        
            PreemptiveStateMachine.add('DropBlue4',
                      DropBlue4(),
                      transitions={'succeeded':'DropBlue5', 'aborted':'Milieu1'}) 
                        
            PreemptiveStateMachine.add('DropBlue5',
                      DropBlue5(),
                      transitions={'succeeded':'DropBlue6', 'aborted':'Milieu1'}) 
                                    
            PreemptiveStateMachine.add('DropBlue6',
                      DropBlue6(),
                      transitions={'succeeded':'DropBlue7', 'aborted':'Milieu1'}) 
            
            PreemptiveStateMachine.add('DropBlue7',
                      DropBlue7(),
                      transitions={'succeeded':'RecalBlue', 'aborted':'Milieu1'}) 
            
            PreemptiveStateMachine.add('RecalBlue',
                      RecalBlue(),
                      transitions={'done':'endOpening'})  
 
            #evitement par le milieu
            PreemptiveStateMachine.add('Milieu1',
                      Milieu1(),
                      transitions={'succeeded':'Milieu2', 'aborted':'endOpening'})  
            PreemptiveStateMachine.add('Milieu2',
                      Milieu2(),
                      transitions={'succeeded':'Milieu3', 'aborted':'endOpening'})  
            PreemptiveStateMachine.add('Milieu3',
                      Milieu3(),
                      transitions={'succeeded':'endOpening', 'aborted':'endOpening'})  
        
############### DESCENTE DE TABLE

class EscapeStartpoint(CyclicActionState):
    def createAction(self):
        Data.obstacleAvoidType='None'
        
        if Data.color=='red':
            pose = AmbiPoseRed(-(3*0.350), Table.HWALL_Y-0.180,0, Data.color)
        else:
            pose = AmbiPoseRed(-(3*0.350), Table.HWALL_Y-0.220,0, Data.color)
        self.pointcap(pose.x, pose.y, pose.theta)

class LigneVert1(CyclicActionState):
    def createAction(self):
        self.dropOnCase(AmbiCaseRed(-4, 4, Data.color))

class LigneVert2(CyclicActionState):
    def createAction(self):
        Data.obstacleAvoidType='Milieu'
        Data.rearObstacleAvoidType='Normal'
        self.dropOnCase(AmbiCaseRed(-4, -2, Data.color))
        
class LigneVert3(CyclicActionState):
    def createAction(self):
        self.dropOnCase(AmbiCaseRed(-1, -3, Data.color))


class SwitchRedBlue(CyclicState):
    def __init__(self):
        CyclicState.__init__(self, outcomes=['red','blue'])
    
    def executeTransitions(self):
        if Data.color=='red':
            return 'red'
        else:
            return 'blue'

############### DEPOSE DU ROUGE EN BAS
      
class DropRed1(CyclicActionState):
    def createAction(self):
        self.cap(-pi/3)

class RecalRed(CyclicState):
    def __init__(self):
        CyclicState.__init__(self, outcomes=['done'])
    
    def executeTransitions(self):
        return 'done'
        
    def executeIn(self):
        self.relocate()

class DropRed2(CyclicActionState):
    def createAction(self):
        self.dropOnCase(Case(-1,-5))
        
class DropRed3(CyclicActionState):
    def createAction(self):
        self.backward(0.300)

################## DEPOSE DU BLEU EN BAS

class DropBlue1(CyclicActionState):
    def createAction(self):
        self.dropOnCase(Case(0,-4))
        
class DropBlue2(CyclicActionState):
    def createAction(self):
        self.cap(-pi/2)
        
        
class DropBlue3(CyclicActionState):
    def createAction(self):
        self.backward(0.300)

class DropBlue4(CyclicActionState):
    def createAction(self):
        self.cap(radians(-125.0))

class DropBlue5(CyclicActionState):
    def createAction(self):
        self.forward(0.350)

class DropBlue6(CyclicActionState):
    def createAction(self):
        self.cap(radians(-85.0)) 
        
class RecalBlue(CyclicState):
    def __init__(self):
        CyclicState.__init__(self, outcomes=['done'])
    
    def executeTransitions(self):
        return 'done'

    def executeIn(self):
        self.relocate()
        
class DropBlue7(CyclicActionState):
    def createAction(self):
        self.backward(0.300)
        
####### ALLER SUR CASE MILIEU
class Milieu1(CyclicActionState):
    def createAction(self):
        Data.obstacleAvoidType='JumpToNext'
        self.dropOnCase(Case(0,0))
        
class Milieu2(CyclicActionState):
    def createAction(self):
        self.dropOnCase(AmbiCaseRed(3,-1,Data.color))

class Milieu3(CyclicActionState):
    def createAction(self):
        self.backward(0.350)     
         
################# PREEMPTIONS

class ObstaclePreemption(FrontObstaclePreempter):
    def __init__(self):
        FrontObstaclePreempter.__init__(self, outcomes=['avoidByMilieu','jump'])

    def executeTransitions(self):
        if Data.obstacleAvoidType=='Milieu':
            return 'avoidByMilieu'
        else:
            return 'jump'

class RearObstaclePreemption(RearObstaclePreempter):
    def __init__(self):
        RearObstaclePreempter.__init__(self, outcomes=['rearobstaclepreemption'])
       
    def executeTransitions(self):
        return 'rearobstaclepreemption' 
    

