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
                      transitions={'avoid':'problem'})
            
            PreemptiveStateMachine.add('EscapeStartpoint',
                      EscapeStartpoint(),
                      transitions={'succeeded':'LigneVert1', 'aborted':'problem'})
            
            self.setInitialState('EscapeStartpoint')
            
            PreemptiveStateMachine.add('LigneVert1',
                      LigneVert1(),
                      transitions={'succeeded':'LigneVert2', 'aborted':'problem'})

            PreemptiveStateMachine.add('LigneVert2',
                      LigneVert2(),
                      transitions={'succeeded':'LigneVert3', 'aborted':'problem'})

            PreemptiveStateMachine.add('LigneVert3',
                      LigneVert3(),
                      transitions={'succeeded':'SwitchRedBlue', 'aborted':'problem'})      
                  
            # attention il y a deux branches, suivant si on est rouge ou bleu
            PreemptiveStateMachine.add('SwitchRedBlue',
                      SwitchRedBlue(),
                      transitions={'red':'DropRed1', 'blue':'DropBlue1'})  
            
            
            PreemptiveStateMachine.add('DropRed1',
                      DropRed1(),
                      transitions={'succeeded':'RecalRed', 'aborted':'problem'})   
            
            PreemptiveStateMachine.add('RecalRed',
                      RecalRed(),
                      transitions={'done':'DropRed2'})   
            
            PreemptiveStateMachine.add('DropRed2',
                      DropRed2(),
                      transitions={'succeeded':'DropRed3', 'aborted':'problem'})   
 
            PreemptiveStateMachine.add('DropRed3',
                      DropRed3(),
                      transitions={'succeeded':'endOpening', 'aborted':'problem'})  
 
             
            PreemptiveStateMachine.add('DropBlue1',
                      DropBlue1(),
                      transitions={'succeeded':'DropBlue2', 'aborted':'problem'})   
            
            PreemptiveStateMachine.add('DropBlue2',
                      DropBlue2(),
                      transitions={'succeeded':'DropBlue3', 'aborted':'problem'}) 
            
            PreemptiveStateMachine.add('DropBlue3',
                      DropBlue3(),
                      transitions={'succeeded':'DropBlue4', 'aborted':'problem'}) 
                        
            PreemptiveStateMachine.add('DropBlue4',
                      DropBlue4(),
                      transitions={'succeeded':'DropBlue5', 'aborted':'problem'}) 
                        
            PreemptiveStateMachine.add('DropBlue5',
                      DropBlue5(),
                      transitions={'succeeded':'DropBlue6', 'aborted':'problem'}) 
                                    
            PreemptiveStateMachine.add('DropBlue6',
                      DropBlue6(),
                      transitions={'succeeded':'DropBlue7', 'aborted':'problem'}) 
            
            PreemptiveStateMachine.add('DropBlue7',
                      DropBlue7(),
                      transitions={'succeeded':'RecalBlue', 'aborted':'problem'}) 
            
            PreemptiveStateMachine.add('RecalBlue',
                      RecalBlue(),
                      transitions={'done':'endOpening'})  
 
############### DESCENTE DE TABLE

class EscapeStartpoint(CyclicActionState):
    def createAction(self):
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
        self.cap(radians(-120.0))

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
         
################# PREEMPTIONS

class ObstaclePreemption(PreemptiveCyclicState):
    def __init__(self):
        PreemptiveCyclicState.__init__(self, outcomes=['avoid'])
        self.blinding_period=rospy.get_param("/blinding_period")

    def preemptionCondition(self):
        #######################
        return False
        #######################
        if Inputs.getobstacle()==1 and rospy.get_rostime().secs-Data.time_obstacle>self.blinding_period:
            Data.time_obstacle=rospy.get_rostime().secs
            return True
        else:
            return False
       
    def executeTransitions(self):
        return 'avoid'

        
class EscapeObstacle(CyclicActionState):
    def createAction(self):
        case = AmbiCaseRed(-3, 3, Data.color)
        cap = AmbiCapRed(0,Data.color)
        self.pointcap_reverse(case.xCenter, case.yCenter, cap.angle)