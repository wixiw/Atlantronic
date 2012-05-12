#!/usr/bin/env python

#libraries for ROS
import roslib; roslib.load_manifest('arp_master')

from arp_master import *

class EndGame(PreemptiveStateMachine):
    def __init__(self):
        PreemptiveStateMachine.__init__(self,outcomes=['endEndGame'])
        with self:
            PreemptiveStateMachine.addPreemptive('EndMatchPreemption',
                                             EndMatchPreempter(0),
                                             transitions={'endMatch':'endEndGame'})
            
            PreemptiveStateMachine.add('FinalDrop',
                       ClawFingerOrder(0.5,0.5,0.5,0.5),
                      transitions={'succeeded':'endEndGame','timeout':'endEndGame'})
            self.setInitialState('FinalDrop')
            
            