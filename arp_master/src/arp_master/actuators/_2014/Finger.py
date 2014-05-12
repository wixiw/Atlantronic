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

#TODO
##
##
## Remettre le preemptive

class FingerMainStateMachine(PreemptiveStateMachine):
    def __init__(self):
        PreemptiveStateMachine.__init__(self,outcomes=['end','problem'])
        with self:      

#Remplacer UserDebugTrigger par vraie etat une fois fini
            
            PreemptiveStateMachine.add('EmptyUpPos',
                      UserDebugTrigger("Position haute vide"),
                      transitions={'continue':'EmptyDownPos'})

            PreemptiveStateMachine.add('EmptyDownPos',
                      UserDebugTrigger("Position basse vide"),
                      transitions={'continue':'FullDownPos'})

            PreemptiveStateMachine.add('FullDownPos',
                      UserDebugTrigger("Attendre Gachette"),
                      transitions={'continue':'FullUpPos'})
            
            PreemptiveStateMachine.add('FullUpPos',
                      UserDebugTrigger("Gachette armee"),
                      transitions={'continue':'Test'})

#Test?

            PreemptiveStateMachine.add('Test',
                      UserDebugTrigger("Presence feu ?"),
                      transitions={'continue':'FullDownDrop'})
            
            PreemptiveStateMachine.add('FullDownDrop',
                      UserDebugTrigger("Position basse pleine depose"),
                      transitions={'continue':'EmptyDownDrop'})
            
            PreemptiveStateMachine.add('EmptyDownDrop',
                      UserDebugTrigger("Position basse vide post depose"),
                      transitions={'continue':'EmptyUpPos'})            
            