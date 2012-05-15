#!/usr/bin/env python

#libraries for ROS
import roslib; roslib.load_manifest('arp_master')
from arp_master import *

from DynamixelActionState import *
from Table2012 import *
from Robot2012 import *

class SweepSweep(PreemptiveStateMachine):
    def __init__(self):
        PreemptiveStateMachine.__init__(self,outcomes=['end','problem'])
        with self:      
            PreemptiveStateMachine.addPreemptive('EndMatchPreemption',
                                             EndMatchPreempter(0.0),
                                             transitions={'endMatch':'end'})
            
            PreemptiveStateMachine.add('Prepare',
                      AmbiOmniDirectOrder(0.750,-0.700,pi/2),
                      transitions={'succeeded':'OpenFingers', 'timeout':'Debloque'})
            self.setInitialState('Prepare')
                
            PreemptiveStateMachine.add('OpenFingers',
                      FingersOnlyState('open'), 
                      transitions={'succeeded':'Sweep1', 'timeout':'Sweep1'})
        
            
            PreemptiveStateMachine.add('Sweep1',
                      AmbiOmniDirectOrder(0.750,0.200,pi/3),
                      transitions={'succeeded':'Rotate1', 'timeout':'Debloque'})
            
            PreemptiveStateMachine.add('Rotate1',
                      AmbiOmniDirectOrder(0.750,0.200,0),
                      transitions={'succeeded':'Push1', 'timeout':'Debloque'})
            
            PreemptiveStateMachine.add('Push1',
                      AmbiOmniDirectOrder(1.000,0.200,0),
                      transitions={'succeeded':'Back1', 'timeout':'Back1'})
            
            PreemptiveStateMachine.add('Back1',
                      AmbiOmniDirectOrder(0.750,0.200,0),
                      transitions={'succeeded':'end', 'timeout':'end'})
            
            #cas d'erreur
            PreemptiveStateMachine.add('Debloque',
                      Replay(1.0),
                      transitions={'succeeded':'UnSetVmax', 'timeout':'UnSetVmax'})
            
            PreemptiveStateMachine.add('UnSetVmax', 
                      SetVMaxState(0.0),
                      transitions={'succeeded':'ExitState','timeout':'ExitState'}) 
            
            PreemptiveStateMachine.add('ExitState',
                      FingerClawState('close'), 
                      transitions={'succeeded':'problem', 'timeout':'problem'})  
            