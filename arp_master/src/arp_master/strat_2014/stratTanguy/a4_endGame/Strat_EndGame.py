#!/usr/bin/env python

#libraries for ROS
import roslib; roslib.load_manifest('arp_master')
from arp_master import *
from arp_master.strat_2014 import *

class EndGame(PreemptiveStateMachine):
    def __init__(self):
        PreemptiveStateMachine.__init__(self,outcomes=['endEndGame'])
        with self:
            PreemptiveStateMachine.addPreemptive('EndMatchPreemption',
                                             EndMatchPreempter(0),
                                             transitions={'endMatch':'endEndGame'})
            
            PreemptiveStateMachine.add('FinalDrop',
                      FinalDrop(),
                      transitions={'succeeded':'endEndGame','timeout':'FinalDrop'})
            self.setInitialState('FinalDrop')

class FinalDrop(CyclicActionState):
    def createAction(self):
        self.cap(pi/2)
            