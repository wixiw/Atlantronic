#!/usr/bin/env python

#libraries for ROS
import roslib; roslib.load_manifest('arp_master')

from arp_master import *

class Opening(PreemptiveStateMachine):
    def __init__(self):
        PreemptiveStateMachine.__init__(self, outcomes=['endOpening', 'problem'])
        with self:
            PreemptiveStateMachine.addPreemptive('EndMatchPreemption',
                                             EndMatchPreempter(-5.0),
                                             transitions={'endMatch':'problem'})
            
            
#            PreemptiveStateMachine.add('SweepSweep',
#                      SweepSweep(), 
#                      transitions={'succeeded':'problem', 'timeout':'problem'})
#            self.setInitialState('SweepSweep')

            PreemptiveStateMachine.add('OpenLeftFinger',
                      FingersOnlyState('open_left'), 
                      transitions={'succeeded':'GotoTopCloseCoin', 'timeout':'GotoTopCloseCoin'})
            self.setInitialState('OpenLeftFinger')
            
            PreemptiveStateMachine.add('GotoTopCloseCoin',
                      AmbiOmniDirectOrder(0.100, 0.600,-pi/2),
                      transitions={'succeeded':'SetStratInfo', 'timeout':'SetStratInfo'})
            
            PreemptiveStateMachine.add('SetStratInfo',
                      SetStratInfoState('topCloseCoinInPosition', False),
                      transitions={'ok':'CleanTopCloseTotem'})
             
            PreemptiveStateMachine.add('CleanTopCloseTotem',
                      CleanTopCloseTotem(),
                      transitions={'endClean':'endOpening', 'problem':'Debloque'})
            
            PreemptiveStateMachine.add('Debloque',
                      Replay(1.0),
                      transitions={'succeeded':'problem', 'timeout':'problem'})
            
 