#!/usr/bin/env python

#libraries for ROS
import roslib; roslib.load_manifest('arp_master')

from arp_master import *

class Endgame(PreemptiveStateMachine):
    def __init__(self):
        PreemptiveStateMachine.__init__(self,outcomes=['endEndgame'])
        with self:
            PreemptiveStateMachine.addPreemptive('EndMatchPreemption',
                                             EndMatchPreempter(0),
                                             transitions={'endMatch':'endEndgame'})
            
            PreemptiveStateMachine.add('FinalDrop',
                      FinalDrop(),
                      transitions={'succeeded':'endEndgame','aborted':'FinalDrop'})
            self.setInitialState('FinalDrop')

class FinalDrop(CyclicActionState):
    def createAction(self):
        self.cap(pi/2)
            