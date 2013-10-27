#!/usr/bin/env python

#libraries for ROS
import roslib; roslib.load_manifest('arp_master')
from arp_master import *
from arp_master.strat_2014 import *

class MiddleGame(PreemptiveStateMachine):
    def __init__(self):
        PreemptiveStateMachine.__init__(self,outcomes=['endMiddleGame'])
        with self:      
            PreemptiveStateMachine.addPreemptive('EndMatchPreemption',
                                             EndMatchPreempter(-5.0),
                                             transitions={'endMatch':'endMiddleGame'})
            # other states
            PreemptiveStateMachine.add('EtatA',
                      AmbiOmniDirectOrder(0.700, -0.700,pi,0.300),
                      transitions={'succeeded':'EtatB', 'timeout':'ReverseOrder'})
            #as initial state is not the preemptive one, it is necessary to add the information here !
            self.setInitialState('EtatA')
            
            PreemptiveStateMachine.add('EtatB',
                      AmbiOmniDirectOrder(0.700,-0.000,pi,0.300),
                      transitions={'succeeded':'EtatA', 'timeout':'ReverseOrder'})
        
            PreemptiveStateMachine.add('ReverseOrder',
                      Rewind(1.0),
                      transitions={'succeeded':'endMiddleGame', 'timeout':'endMiddleGame'})


         
