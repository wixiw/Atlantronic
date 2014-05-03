#! /usr/bin/env python
# -*- coding: latin-1 -*-

#libraries for ROS
import roslib; roslib.load_manifest('arp_master')

from arp_master import *
import os
from Waiting import *

from arp_master.fsmFramework import *
from arp_master.commonStates import *

class Unblock(PreemptiveStateMachine):
    def __init__(self):
        PreemptiveStateMachine.__init__(self, outcomes=['succeeded', 'nearlyEndMatch'])
        with self:
            PreemptiveStateMachine.addPreemptive('EndMatchPreemption',
                      EndMatchPreempter(-5.0),
                      transitions={'endMatch':'nearlyEndMatch'})
            
            PreemptiveStateMachine.add('WaitProblemToLeaveWithLuck',
                      WaiterState(2.0),
                      transitions={'timeout':'Rewind'})
            self.setInitialState('WaitProblemToLeaveWithLuck')
            
            PreemptiveStateMachine.add('Rewind',
                      Rewind(2.0),
                      transitions={'succeeded':'succeeded', 'timeout':'RandomMove'})
            
            PreemptiveStateMachine.add('RandomMove',
                      RandomMove(0.5),
                      transitions={'succeeded':'succeeded', 'timeout':'Rewind'})