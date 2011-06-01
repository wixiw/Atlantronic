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
from arp_master.strat.util.Inputs import Inputs
from arp_master.strat.util.Data import Data
from arp_ods.msg import OrderGoal
from arp_ods.msg import OrderAction

import SubStrat_GetPionBord
import SubStrat_DropPion

from math import pi

from arp_master.strat.util.Table2011 import *
from arp_master.strat.util.UtilARD import *

class Middlegame_D(PreemptiveStateMachine):
    def __init__(self):
        PreemptiveStateMachine.__init__(self,outcomes=['endMiddlegame'])
        with self:
            #preemptive states
            PreemptiveStateMachine.addPreemptive('ObstaclePreemption',
                                             ObstaclePreemption(),
                                             transitions={'obstaclepreemption':'Selector'})
            PreemptiveStateMachine.addPreemptive('RearObstaclePreemption',
                                             RearObstaclePreemption(),
                                             transitions={'rearobstaclepreemption':'Selector'})
            
            PreemptiveStateMachine.addPreemptive('EndMatchPreemption',
                                             EndMatchPreemption(),
                                             transitions={'endPreemption':'endMiddlegame'})
            # other states
            PreemptiveStateMachine.add('Selector',
                      Selector(),
                      transitions={'getpion':'GetPionBord'})
            #as initial state is not the preemptive one, it is necessary to add the information here !
            self.setInitialState('Selector')
            
            PreemptiveStateMachine.add('GetPionBord', SubStrat_GetPionBord.GetPionBord(),
                                   transitions={'got':'DropPion','obstacle':'Selector','endmatch':'endMiddlegame','problem':'Selector'})
            PreemptiveStateMachine.add('DropPion', SubStrat_DropPion.DropPion(),
                                   transitions={'dropped':'Selector','obstacle':'Selector','endmatch':'endMiddlegame'})


#l'etat qui decide de ce qui va etre fait maintenant
class Selector(CyclicState):
    def __init__(self):
        CyclicState.__init__(self, outcomes=['getpion'])
    
    def executeTransitions(self):
        if Data.pionBordObjectif==None:
            Data.pionBordObjectif=PionBord(0,Data.color)
        else:
            Data.pionBordObjectif=PionBord((Data.pionBordObjectif.rang+1)%4,Data.color)
            
        return 'getpion'
        

    
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
        self.blinding_period=rospy.get_param("/blinding_period")
       
    def executeTransitions(self):
        return 'obstaclepreemption'
    
class RearObstaclePreemption(RearObstaclePreempter):
    def __init__(self):
        RearObstaclePreempter.__init__(self, outcomes=['rearobstaclepreemption'])
       
    def executeTransitions(self):
        return 'rearobstaclepreemption'