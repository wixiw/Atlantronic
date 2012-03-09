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
from arp_master.strat.stratVierge.util.WaiterState import WaiterState
from arp_master.strat.stratVierge.util.TableVierge import *
from arp_master.strat.stratVierge.util.UtilARD import *

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