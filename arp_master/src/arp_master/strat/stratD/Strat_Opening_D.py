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
from arp_master.strat.util.EndMatchPreempter import EndMatchPreempter
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
            PreemptiveStateMachine.addPreemptive('EndMatchPreemption',
                                             EndMatchPreempter(-30.0),
                                             transitions={'endMatch':'problem'})
            
            PreemptiveStateMachine.addPreemptive('ObstaclePreemption',
                      ObstaclePreemption(),
                      transitions={'avoidByMilieu':'WaitBeforeMilieu','jump':'WaitBeforeJump'})
            
            PreemptiveStateMachine.add('WaitBeforeMilieu',
                      WaiterState(1.0),
                      transitions={'done':'Reverse1'})
            
            PreemptiveStateMachine.add('Reverse1',
                      Reverse1(),
                      transitions={'succeeded':'Milieu1', 'aborted':'Milieu1'})
            
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
                      transitions={'succeeded':'LigneVert11', 'aborted':'Reverse1'})
            
            PreemptiveStateMachine.add('LigneVert11',
                      LigneVert11(),
                      transitions={'succeeded':'LigneVert2', 'aborted':'Reverse1'})

            PreemptiveStateMachine.add('LigneVert2',
                      LigneVert2(),
                      transitions={'succeeded':'SwitchBlackBox', 'aborted':'Reverse1'})
            
            PreemptiveStateMachine.add('SwitchBlackBox',
                      SwitchRedBlue(),
                      transitions={'red':'SwitchRedBlue', 'blue':'BlackBox1'})
                        
            PreemptiveStateMachine.add('BlackBox1',
                      BlackBox1(),
                      transitions={'succeeded':'SwitchRedBlue', 'aborted':'BlackBox2'})
            
            PreemptiveStateMachine.add('BlackBox2',
                      BlackBox2(),
                      transitions={'succeeded':'BlackBox3', 'aborted':'Reverse1'})
                        
            PreemptiveStateMachine.add('BlackBox3',
                      BlackBox3(),
                      transitions={'succeeded':'BlackBox4', 'aborted':'Reverse1'})
            
            PreemptiveStateMachine.add('BlackBox4',
                      BlackBox4(),
                      transitions={'succeeded':'SwitchRedBlue', 'aborted':'Reverse1'})
    
                  
            # attention il y a deux branches, suivant si on est rouge ou bleu
            #rouge
            PreemptiveStateMachine.add('SwitchRedBlue',
                      SwitchRedBlue(),
                      transitions={'red':'DropRed0', 'blue':'DropBlue_1'})  
            
            PreemptiveStateMachine.add('DropRed0',
                      DropRed0(),
                      transitions={'succeeded':'DropRed1', 'aborted':'Reverse1'})
            
            PreemptiveStateMachine.add('DropRed1',
                      DropRed1(),
                      transitions={'succeeded':'DropRed11', 'aborted':'Reverse1'})   
            
            PreemptiveStateMachine.add('DropRed11',
                      DropRed11(),
                      transitions={'succeeded':'DropRed2', 'aborted':'Reverse1'}) 
            
            PreemptiveStateMachine.add('DropRed2',
                      DropRed2(),
                      transitions={'succeeded':'DropRed3', 'aborted':'Reverse1'})   
 
            PreemptiveStateMachine.add('DropRed3',
                      DropRed3(),
                      transitions={'succeeded':'Photo', 'aborted':'Reverse1'})  
 
            #bleu 
            PreemptiveStateMachine.add('DropBlue_1',
                      DropBlue_1(),
                      transitions={'succeeded':'DropBlue0', 'aborted':'Reverse1'}) 
            
            PreemptiveStateMachine.add('DropBlue0',
                      DropBlue0(),
                      transitions={'succeeded':'DropBlue1', 'aborted':'Reverse1'})  
            PreemptiveStateMachine.add('DropBlue1',
                      DropBlue1(),
                      transitions={'succeeded':'DropBlue2', 'aborted':'Reverse1'})   
            
            PreemptiveStateMachine.add('DropBlue2',
                      DropBlue2(),
                      transitions={'succeeded':'DropBlue3', 'aborted':'Reverse1'}) 
            
            PreemptiveStateMachine.add('DropBlue3',
                      DropBlue3(),
                      transitions={'succeeded':'DropBlue4', 'aborted':'Reverse1'}) 
                        
            PreemptiveStateMachine.add('DropBlue4',
                      DropBlue4(),
                      transitions={'succeeded':'DropBlue5', 'aborted':'Reverse1'}) 
                        
            PreemptiveStateMachine.add('DropBlue5',
                      DropBlue5(),
                      transitions={'succeeded':'DropBlue6', 'aborted':'Reverse1'}) 
                                    
            PreemptiveStateMachine.add('DropBlue6',
                      DropBlue6(),
                      transitions={'succeeded':'Photo', 'aborted':'Reverse1'}) 
            
            #la petite photo
            PreemptiveStateMachine.add('Photo',
                      Photo(),
                      transitions={'succeeded':'FindRoi', 'aborted':'endOpening'}) 
            
            PreemptiveStateMachine.add('FindRoi',
                      FindRoi(),
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
                      transitions={'succeeded':'Photo', 'aborted':'endOpening'})  
        
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
        
class LigneVert11(CyclicActionState):
    def createAction(self):
        self.dropOnCase(AmbiCaseRed(-4, 2, Data.color))

class LigneVert2(CyclicActionState):
    def createAction(self):
        Data.obstacleAvoidType='Milieu'
        Data.rearObstacleAvoidType='Normal'
        self.dropOnCase(AmbiCaseRed(-4, -2, Data.color))

class BlackBox1(CyclicActionState):
    def createAction(self):
        self.forward(0.120)
        
class BlackBox2(CyclicActionState):
    def createAction(self):
        self.dropOnCase(AmbiCaseRed(-2, -3, Data.color))
        
class BlackBox3(CyclicActionState):
    def createAction(self):
        self.cap(-pi/3)

class BlackBox4(CyclicActionState):
    def createAction(self):
        self.cap(0)

class SwitchRedBlue(CyclicState):
    def __init__(self):
        CyclicState.__init__(self, outcomes=['red','blue'])
    
    def executeTransitions(self):
        if Data.color=='red':
            return 'red'
        else:
            return 'blue'

############### DEPOSE DU ROUGE EN BAS
class DropRed0(CyclicActionState):
     def createAction(self):
         self.dropOnCase(AmbiCaseRed(-1, -3, Data.color))
               
class DropRed1(CyclicActionState):
    def createAction(self):
        self.cap(radians(-105))
        
class DropRed11(CyclicActionState):
    def createAction(self):
        self.cap(radians(-60))

class DropRed2(CyclicActionState):
    def createAction(self):
        self.dropOnCase(Case(-1,-5))
        
class DropRed3(CyclicActionState):
    def createAction(self):
        self.backward(0.300)

################## DEPOSE DU BLEU EN BAS

class DropBlue_1(CyclicActionState):
    def createAction(self):
        self.dropOnCase(AmbiCaseRed(0, -2, Data.color))
        
class DropBlue0(CyclicActionState):
    def createAction(self):
        self.cap(-radians(70))

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
        self.forward(0.320)

class DropBlue6(CyclicActionState):
    def createAction(self):
        self.cap(radians(22.0)) 
        #opitmise pour point de photo
        
        
###### UNE PETITE PHOTO POUR BORIS !
class Photo(CyclicActionState):
    def createAction(self):
        pose = AmbiPoseRed(-0.350, -0.350,3*pi/4, Data.color)
        self.pointcap(pose.x, pose.y, pose.theta)
        
class FindRoi(CyclicState):
    def __init__(self):
        CyclicState.__init__(self, outcomes=['done'])
    
    def executeTransitions(self):
        result=self.findRoyalFamily()
        return 'done'

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
    
################# REVERSER
class Reverse1(CyclicActionState):
    def createAction(self):
        order=Data.listReplayOrders.pop()#retourne le dernier element et l'enleve de la liste
        if order==None:
            self.dropOnCase(Case(0,0))
        self.executeReplayOrder(order)
