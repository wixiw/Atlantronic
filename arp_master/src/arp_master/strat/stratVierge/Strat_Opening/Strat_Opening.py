#!/usr/bin/env python

import roslib; roslib.load_manifest('arp_master')
import rospy
import smach
import smach_ros
import smach_msgs

from arp_master.strat.stratVierge.util.CyclicState import CyclicState
from arp_master.strat.stratVierge.util.CyclicActionState import CyclicActionState
from arp_master.strat.stratVierge.util.Inputs import Inputs
from arp_master.strat.stratVierge.util.Data import Data
from arp_master.strat.stratVierge.util.EndMatchPreempter import EndMatchPreempter
from arp_master.strat.stratVierge.util.PreemptiveStateMachine import PreemptiveStateMachine
from arp_master.strat.stratVierge.util.PreemptiveCyclicState import PreemptiveCyclicState
from arp_master.strat.stratVierge.util.ObstaclePreempter import FrontObstaclePreempter
from arp_master.strat.stratVierge.util.ObstaclePreempter import RearObstaclePreempter
from arp_master.strat.stratVierge.util.WaiterState import WaiterState



from math import *

from arp_master.strat.stratVierge.util.TableVierge import *
from arp_master.strat.stratVierge.util.UtilARD import *


class Opening(PreemptiveStateMachine):
    def __init__(self):
        PreemptiveStateMachine.__init__(self, outcomes=['endOpening', 'problem'])
        with self:
            PreemptiveStateMachine.addPreemptive('EndMatchPreemption',
                                             EndMatchPreempter(-5.0),
                                             transitions={'endMatch':'problem'})
            
            PreemptiveStateMachine.add('GotoMilieu',
                      GotoMilieu(),
                      transitions={'succeeded':'WaitBeforeNext', 'aborted':'ReverseOrder'})
            
            self.setInitialState('GotoMilieu')
            
            PreemptiveStateMachine.add('ReverseOrder',
                      GotoMilieu(),
                      transitions={'succeeded':'GotoMilieu', 'aborted':'GotoMilieu'})
            
            PreemptiveStateMachine.add('WaitBeforeNext',
                      WaiterState(1.0),
                      transitions={'done':'endOpening'})
                        
            
        
############### Ordres de motion

class GotoMilieu(CyclicActionState):
    def createAction(self):
        pose = AmbiPoseRed(-(3*0.350), Table.HWALL_Y-0.180,0, Data.color)
        self.pointcap(pose.x, pose.y, pose.theta)

         
################# REVERSER
class ReverseOrder(CyclicActionState):
    def createAction(self):
        order=Data.listReplayOrders.pop() #retourne le dernier element et l'enleve de la liste
        self.executeReplayOrder(order)
