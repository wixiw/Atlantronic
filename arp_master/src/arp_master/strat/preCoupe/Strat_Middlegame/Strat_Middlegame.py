#!/usr/bin/env python

import roslib; roslib.load_manifest('arp_master')
import rospy
from rospy import loginfo
import smach
import smach_ros
import smach_msgs

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




class Middlegame(PreemptiveStateMachine):
    def __init__(self):
        PreemptiveStateMachine.__init__(self,outcomes=['endMiddlegame'])
        with self:      
            PreemptiveStateMachine.addPreemptive('EndMatchPreemption',
                                             EndMatchPreempter(-5.0),
                                             transitions={'endMatch':'endMiddlegame'})

            PreemptiveStateMachine.add('WaitBeforeNext',
                      WaiterState(1.0),
                      transitions={'done':'endMiddlegame'})
            #as initial state is not the preemptive one, it is necessary to add the information here !
            self.setInitialState('WaitBeforeNext')


############### Ordres de motion

class EtatA(CyclicActionState):
    def createAction(self):
        pose = AmbiPoseRed(0.7, 0.7,0, Data.color)
        self.pointcap(pose.x, pose.y, pose.theta)

class EtatB(CyclicActionState):
    def createAction(self):
        pose = AmbiPoseRed(-0.7,-0.7,pi, Data.color)
        self.pointcap(pose.x, pose.y, pose.theta)
         
################# REVERSER
class ReverseOrder(CyclicActionState):
    def createAction(self):
        order=Data.listReplayOrders.pop() #retourne le dernier element et l'enleve de la liste
        self.executeReplayOrder(order) 
