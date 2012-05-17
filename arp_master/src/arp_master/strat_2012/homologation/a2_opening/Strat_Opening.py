#!/usr/bin/env python

#libraries for ROS
import roslib; roslib.load_manifest('arp_master')

from arp_master import *

class Opening(PreemptiveStateMachine):
    def __init__(self):
        PreemptiveStateMachine.__init__(self, outcomes=['endOpening', 'problem'])
        with self:
            PreemptiveStateMachine.addPreemptive('EndMatchPreemption',
                                             EndMatchPreempter(-Robot2012.END_GAME_DELAY),
                                             transitions={'endMatch':'problem'})
            
            
#            PreemptiveStateMachine.add('Move1',
#                      AmbiOmniDirectOrder(-0.850, 0.750,-pi),
#                      transitions={'succeeded':'Move2', 'timeout':'Debloque'})
#            PreemptiveStateMachine.add('Move2',
#                      AmbiOmniDirectOrder(0.850, 0.750,-pi),
#                      transitions={'succeeded':'Move1', 'timeout':'Debloque'})
            
            
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
                      transitions={'ok':'endOpening'})
             
            #cas d'erreur
            PreemptiveStateMachine.add('Debloque',
                      DeblocReloc(),
                      transitions={'endDeblocReloc':'problem'})
            
 
