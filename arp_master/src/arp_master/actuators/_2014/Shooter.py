#!/usr/bin/env python
# -*- coding: latin-1 -*-

#libraries for ROS
import roslib; roslib.load_manifest('arp_master')

from arp_master.strat_2014.common_2014.Robot2014 import *

from arp_master.fsmFramework import *
from arp_master.commonStates import *
from arp_master.strat_2014 import *
from arp_master.actuators import *

#TODO
##
##
## Remettre le preemptive

class ShooterMainStateMachine(PreemptiveStateMachine):
    def __init__(self):
        PreemptiveStateMachine.__init__(self,outcomes=['end','problem'])
        with self:      

#Remplacer UserDebugTrigger par vraie etat une fois fini
            
            PreemptiveStateMachine.add('ReadyToShoot',
                      UserDebugTrigger("Pret a tirer"),
                      transitions={'continue':'EmptyCannon'})

            PreemptiveStateMachine.add('EmptyCannon',
                      UserDebugTrigger("Canon est vide"),
                      transitions={'continue':'WaitTrigger'})

            PreemptiveStateMachine.add('WaitTrigger',
                      UserDebugTrigger("Attendre Gachette"),
                      transitions={'continue':'TriggerArmed'})
            
            PreemptiveStateMachine.add('TriggerArmed',
                      UserDebugTrigger("Gachette armee"),
                      transitions={'continue':'WaitForBall'})

            PreemptiveStateMachine.add('WaitForBall',
                      UserDebugTrigger("Attendre chute balle"),
                      transitions={'continue':'ReadyToShoot'})