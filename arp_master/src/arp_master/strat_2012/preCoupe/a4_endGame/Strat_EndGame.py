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
            
            PreemptiveStateMachine.add('EndGameBip',
                      EndGameBip(),
                      transitions={'end':'endEndGame'})
            #as initial state is not the preemptive one, it is necessary to add the information here !
            self.setInitialState('EndGameBip')

            
class EndGameBip(CyclicState):
    def __init__(self):
        CyclicState.__init__(self, outcomes=['end'])
    
    def executeIn(self):
        os.system("beep -f 300 -l100 -r2") 
    
    def executeTransitions(self):
            return 'end'
