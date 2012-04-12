#!/usr/bin/env python

import roslib; roslib.load_manifest('arp_master')
import rospy
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

class Uninitialisation(smach.StateMachine):
    def __init__(self):
        smach.StateMachine.__init__(self,outcomes=['endUninitialisation'])
        with self:
            smach.StateMachine.add('UninitialisationState',
                      UninitialisationState(),
                      transitions={'ok':'endUninitialisation'})
      
class UninitialisationState(CyclicState):
    def __init__(self):
        CyclicState.__init__(self, outcomes=['ok'])

    def executeTransitions(self):
        return 'ok'     
    
    
#the state that will wait for the start to be pluged
class WaitForStart(CyclicState):
    def __init__(self):
        CyclicState.__init__(self, outcomes=['end'])
    
    def executeIn(self):
        self.result = self.disablePower()
        
    def executeTransitions(self):
            return 'end'
