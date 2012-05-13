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
            
            PreemptiveStateMachine.add('OpenLeftFinger',
                      FingersOnlyState('open_left'), 
                      transitions={'succeeded':'GotoTopCloseCoin', 'timeout':'GotoTopCloseCoin'})
            self.setInitialState('OpenLeftFinger')
            
            PreemptiveStateMachine.add('GotoTopCloseCoin',
                      AmbiOmniDirectOrder(0.500, 0.703,-pi/2),
                      transitions={'succeeded':'SetStratInfo', 'timeout':'Debloque'})
            
            PreemptiveStateMachine.add('SetStratInfo',
                      SetStratInfoState('topCloseCoinInPosition', False),
                      transitions={'ok':'OpenRightFinger'})
            
            PreemptiveStateMachine.add('OpenRightFinger',
                      FingersOnlyState('open'), 
                      transitions={'succeeded':'GotoFirstTotemCoin', 'timeout':'GotoTopCloseCoin'})
            
            PreemptiveStateMachine.add('GotoFirstTotemCoin',
                      AmbiOmniDirectOrder_cpoint(0.060,0.0,0.0,
                                                 0.130, 0.370,-pi/4),
                      transitions={'succeeded':'TotemClean', 'timeout':'Debloque'})
            
            PreemptiveStateMachine.add('TotemClean',
                      AmbiOmniDirectOrder_cpoint(0.162, -0.173,0.0,
                                                 0.600,0.140,-pi/3),
                      transitions={'succeeded':'Push', 'timeout':'Debloque'})
            
            PreemptiveStateMachine.add('Push',
                      AmbiOmniDirectOrder_cpoint(0.162, -0.173,0.0,
                                                 0.650,0.000,-pi/3),
                      transitions={'succeeded':'Back', 'timeout':'Debloque'})
            
            PreemptiveStateMachine.add('Back',
                      AmbiOmniDirectOrder(0.500,0.500,-pi/3),
                      transitions={'succeeded':'CloseFingers', 'timeout':'Debloque'})
            
            PreemptiveStateMachine.add('CloseFingers',
                      FingersOnlyState('close'), 
                      transitions={'succeeded':'endOpening', 'timeout':'endOpening'})
            
            
            PreemptiveStateMachine.add('Debloque',
                      Replay(1.0),
                      transitions={'succeeded':'problem', 'timeout':'problem'})
