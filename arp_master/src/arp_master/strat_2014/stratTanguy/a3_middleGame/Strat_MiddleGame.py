#!/usr/bin/env python

#libraries for ROS
import roslib; roslib.load_manifest('arp_master')

from arp_master import *
from arp_master.strat_2014 import *
from arp_master.commonStates import *
from a3_middleGame import ActionSelector

class MiddleGame(PreemptiveStateMachine):
    def __init__(self):
        PreemptiveStateMachine.__init__(self,outcomes=['endMiddleGame', 'motionBlocked'])
        with self:      
            PreemptiveStateMachine.addPreemptive('EndMatchPreemption',
                                             EndMatchPreempter(-5.0),
                                             transitions={'endMatch':'endMiddleGame', })

# Go to Red Fire Bot
#            PreemptiveStateMachine.add('GoToRFB',
#                      AmbiOmniDirectOrder2(Pose2D(-0.400 - Robot2014.FRONT_SIDE.x, -0.600, 0), vmax=1.0),
#                      transitions={'succeeded':'PickRFB', 'timeout':'ReverseOrder'})
            
#as initial state is not the preemptive one, it is necessary to add the information here !

            PreemptiveStateMachine.add('ActionSelector',
                      ActionSelector(),
                      transitions={'succeeded':'DummyAction', 'nearlyEndMatch':'endMiddleGame'})
            self.setInitialState('ActionSelector')
            
            PreemptiveStateMachine.add('DummyAction',
                      WaiterState(2.0),
                      transitions={'timeout':'ActionSelector'})