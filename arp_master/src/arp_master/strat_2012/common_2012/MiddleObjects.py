#!/usr/bin/env python

#libraries for ROS
import roslib; roslib.load_manifest('arp_master')
from arp_master import *

from DynamixelActionState import *
from Table2012 import *
from Robot2012 import *

class MiddleObjects(PreemptiveStateMachine):
    def __init__(self):
        PreemptiveStateMachine.__init__(self,outcomes=['end','problem'])
        with self:      
            PreemptiveStateMachine.addPreemptive('EndMatchPreemption',
                                             EndMatchPreempter(-5.0),
                                             transitions={'endMatch':'end'})
            
            PreemptiveStateMachine.add('HalfOpenFingers',
                      FingersOnlyState('half_close'), 
                      transitions={'succeeded':'Prepare', 'timeout':'Prepare'})
            self.setInitialState('HalfOpenFingers')
            
            PreemptiveStateMachine.add('Prepare',
                      AmbiOmniDirectOrder(0.825,-0.400,pi),
                      transitions={'succeeded':'OpenFinger', 'timeout':'Debloque'})
            
            PreemptiveStateMachine.add('OpenFinger',
                      FingersOnlyState('open_right'), 
                      transitions={'succeeded':'EngageCoins', 'timeout':'EngageCoins'})
            
            PreemptiveStateMachine.add('EngageCoins',
                      AmbiOmniDirectOrder(0.200,-0.800,3*pi/4),
                      transitions={'succeeded':'PushCoins', 'timeout':'Debloque'})
            
            PreemptiveStateMachine.add('PushCoins',
                      AmbiOmniDirectOrder(-0.100,-0.800,3*pi/4),
                      transitions={'succeeded':'PassThrought', 'timeout':'Debloque'})
            
            PreemptiveStateMachine.add('PassThrought',
                      AmbiOmniDirectOrder(-0.300,-0.500,pi/4),
                      transitions={'succeeded':'SetStratInfo', 'timeout':'Debloque'})
            
            PreemptiveStateMachine.add('SetStratInfo',
                      SetStratInfoState("middleCoinsInPosition", False),
                      transitions={'ok':'end'})
            
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
                      AmbiOmniDirectOrder(-0.800,-0.360,pi/4),
                      transitions={'succeeded':'Moissbat', 'timeout':'Debloque'})
            
            PreemptiveStateMachine.add('Moissbat',
                      AmbiOmniDirectOrder(0.600,-0.360,0),
                      transitions={'succeeded':'end', 'timeout':'Debloque'})
            
         
            PreemptiveStateMachine.add('Debloque',
                      Replay(1.0),
                      transitions={'succeeded':'problem', 'timeout':'problem'})       