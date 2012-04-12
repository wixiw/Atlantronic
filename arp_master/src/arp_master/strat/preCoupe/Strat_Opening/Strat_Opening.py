#!/usr/bin/env python

import roslib; roslib.load_manifest('arp_master')
import rospy
import smach
import smach_ros
import smach_msgs
import os

from arp_master.strat.util.CyclicState import CyclicState
from arp_master.strat.util.CyclicActionState import CyclicActionState
from arp_master.strat.util.Inputs import Inputs
from arp_master.strat.util.Data import Data
from arp_master.strat.util.EndMatchPreempter import EndMatchPreempter
from arp_master.strat.util.PreemptiveStateMachine import PreemptiveStateMachine
from arp_master.strat.util.PreemptiveCyclicState import PreemptiveCyclicState
from arp_master.strat.util.WaiterState import WaiterState
from arp_master.strat.util.TableVierge import *
from arp_master.strat.util.UtilARD import *

from math import *




class Opening(PreemptiveStateMachine):
    def __init__(self):
        PreemptiveStateMachine.__init__(self, outcomes=['endOpening', 'problem'])
        with self:
            PreemptiveStateMachine.addPreemptive('EndMatchPreemption',
                                             EndMatchPreempter(-5.0),
                                             transitions={'endMatch':'problem'})
            
            PreemptiveStateMachine.add('GotoOutStartPos',
                      GotoOutStartPos(),
                      transitions={'succeeded':'TurnToGoldBar', 'aborted':'problem'})
            
            self.setInitialState('GotoOutStartPos')
            
            PreemptiveStateMachine.add('TurnToGoldBar',
                      TurnToGoldBar(),
                      transitions={'succeeded':'GoToGoldBar', 'aborted':'problem'})
            
            PreemptiveStateMachine.add('GoToGoldBar',
                      GoToGoldBar(),
                      transitions={'succeeded':'PushGoldBar', 'aborted':'problem'})
            
            PreemptiveStateMachine.add('PushGoldBar',
                      PushGoldBar(),
                      transitions={'succeeded':'WaitBeforeNext', 'aborted':'problem'})
            
            PreemptiveStateMachine.add('WaitBeforeNext',
                      WaiterState(1.0),
                      transitions={'done':'endOpening'})
                        
            
        
############### Ordres de motion


## oui oui on peut faire plus court et supprimer la rotation pure, mais c'est pour tester des cas et commencer par le plus simple !
class GotoOutStartPos(CyclicActionState):
    def createAction(self):
        pose = AmbiPoseRed(0.750,0.700, -pi, Data.color)
        self.omnidirect(pose.x, pose.y, pose.theta)


class TurnToGoldBar(CyclicActionState):
    def createAction(self):
        self.omnicap(AmbiCapRed(-pi/2, Data.color).angle)
        
        
class GoToGoldBar(CyclicActionState):
    def createAction(self):
        pose = AmbiPoseRed(0.900,0, 0, Data.color)
        self.omnidirect(pose.x, pose.y, pose.theta)

class PushGoldBar(CyclicActionState):
    def createAction(self):
        pose = AmbiPoseRed(1.200,0, 0, Data.color)
        self.omnidirect(pose.x, pose.y, pose.theta)
    
    def executeOut(self):
        os.system("beep -f 300 -l100 -r1") 
