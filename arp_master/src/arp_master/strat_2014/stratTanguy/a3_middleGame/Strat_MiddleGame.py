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

# Go to Red Fire Bot
            PreemptiveStateMachine.add('GoToRFB',
                      AmbiOmniDirectOrder2(-0.400 - Robot2014.FRONT_SIDE.x, -0.600, 0, 1.0),
                      transitions={'succeeded':'PickRFB', 'timeout':'ReverseOrder'})
            
#as initial state is not the preemptive one, it is necessary to add the information here !

            self.setInitialState('GoToRFB')

# Pick Red Fire Bot
            PreemptiveStateMachine.add('PickRFB',
                      WaiterState(1.5),
                      transitions={'timeout':'EtatB'})
                        
            PreemptiveStateMachine.add('EtatB',
                      AmbiOmniDirectOrder2(0.700,-0.000,pi,0.300),
                      transitions={'succeeded':'PickRFB', 'timeout':'ReverseOrder'})
        
            PreemptiveStateMachine.add('ReverseOrder',
                      Rewind(1.0),
                      transitions={'succeeded':'endMiddleGame', 'timeout':'endMiddleGame'})