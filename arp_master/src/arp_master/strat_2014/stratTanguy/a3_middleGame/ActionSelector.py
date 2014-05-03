#! /usr/bin/env python
# -*- coding: latin-1 -*-

#libraries for ROS
import roslib; roslib.load_manifest('arp_master')
from arp_master import *

import os
from arp_master.strat_2014 import *

from arp_master.fsmFramework import *


class ActionSelector(smach.StateMachine):
    def __init__(self):
        smach.StateMachine.__init__(self, outcomes=['succeeded', 'nearlyEndMatch'])
        with self:
           
            smach.StateMachine.add('DummyState',
                      WaiterState(2.0),
                      transitions={'timeout':'succeeded'})