#!/usr/bin/env python
# -*- coding: latin-1 -*-

#libraries for ROS
import roslib; roslib.load_manifest('arp_master')

from arp_master.strat_2014.common_2014.Robot2014 import *

from arp_master.util import *
from arp_master.fsmFramework import *
from arp_master.commonStates import *
from arp_master.strat_2014 import *
from arp_master.actuators import *


#This state machine is configuring all torques on dynamixels
class ActuatorConfigStateMachine(smach.StateMachine):
    def __init__(self, p_side):
        PreemptiveStateMachine.__init__(self,outcomes=['done'])

        
        with self:      

            PreemptiveStateMachine.add('LeftFinger',
                      FingerTorqueConfig('Left', 30),
                      transitions={'done':'RightFinger'})

            PreemptiveStateMachine.add('RightFinger',
                      FingerTorqueConfig('Right', 30),
                      transitions={'done':'LeftCannon'})
            
            PreemptiveStateMachine.add('LeftCannon',
                      CannonBiTorqueConfig('Left', 30, 30),
                      transitions={'done':'RightCannon'})
            
            PreemptiveStateMachine.add('RightCannon',
                      CannonBiTorqueConfig('Right', 30, 30),
                      transitions={'done':'done'})
                        