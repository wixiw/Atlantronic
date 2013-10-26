#!/usr/bin/env python

#libraries for ROS
import roslib; roslib.load_manifest('arp_master')
from arp_master import *
from arp_master.strat_2014 import *

class Opening(PreemptiveStateMachine):
    def __init__(self):
        PreemptiveStateMachine.__init__(self, outcomes=['endOpening', 'problem'])
        with self:
            PreemptiveStateMachine.addPreemptive('EndMatchPreemption',
                                             EndMatchPreempter(-5.0),
                                             transitions={'endMatch':'problem'})
            
            PreemptiveStateMachine.add('EscapeStartArea',
                      AmbiOmniDirectOrder(1.300 - Robot2014.FRONT_SIDE.x,0.400, 0),
                      transitions={'succeeded':'WaitBeforeNext', 'timeout':'Debloque'})
            
            self.setInitialState('EscapeStartArea')
            
            PreemptiveStateMachine.add('Debloque',
                      Rewind(1.0),
                      transitions={'succeeded':'EscapeStartArea', 'timeout':'EscapeStartArea'})
            
            PreemptiveStateMachine.add('WaitBeforeNext',
                      WaiterState(1.0),
                      transitions={'timeout':'endOpening'})
                        
            
    
         

