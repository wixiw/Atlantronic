#!/usr/bin/env python

#libraries for ROS
import roslib; roslib.load_manifest('arp_master')
from arp_master import *
from arp_master.strat_2014 import *

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