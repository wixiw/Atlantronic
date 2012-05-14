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
            
            PreemptiveStateMachine.add('Debloque',
                      Replay(1.0),
                      transitions={'succeeded':'problem', 'timeout':'problem'})
            
     
     
class BackFromMiddleObjects(PreemptiveStateMachine):
    def __init__(self):
        PreemptiveStateMachine.__init__(self,outcomes=['end','problem'])
        with self:      
            PreemptiveStateMachine.addPreemptive('EndMatchPreemption',
                                             EndMatchPreempter(-5.0),
                                             transitions={'endMatch':'end'})
            
            PreemptiveStateMachine.add('OpenFingers',
                      FingersOnlyState('open'), 
                      transitions={'succeeded':'EngageFarBotTotem', 'timeout':'EngageFarBotTotem'})
            self.setInitialState('OpenFingers')
            
            PreemptiveStateMachine.add('EngageFarBotTotem',
                      AmbiOmniDirectOrder(-0.800,-0.380,0),
                      transitions={'succeeded':'Moissbat', 'timeout':'Debloque'})
            
            PreemptiveStateMachine.add('Moissbat',
                      AmbiOmniDirectOrder(0.600,-0.400,0),
                      transitions={'succeeded':'end', 'timeout':'Debloque'})
            
         
            PreemptiveStateMachine.add('Debloque',
                      Replay(1.0),
                      transitions={'succeeded':'problem', 'timeout':'problem'})       