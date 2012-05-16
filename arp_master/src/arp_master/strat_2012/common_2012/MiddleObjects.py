#!/usr/bin/env python

#libraries for ROS
import roslib; roslib.load_manifest('arp_master')
from arp_master import *

from DynamixelActionState import *
from DeblocReloc import *
from Table2012 import *
from Robot2012 import *

class MiddleObjects(PreemptiveStateMachine):
    def __init__(self):
        PreemptiveStateMachine.__init__(self,outcomes=['end','problem'])
        with self:      
            PreemptiveStateMachine.addPreemptive('EndMatchPreemption',
                                             EndMatchPreempter(-5.0),
                                             transitions={'endMatch':'end'})
            
            PreemptiveStateMachine.add('HalfCloseFingers',
                      FingersOnlyState('half_close'), 
                      transitions={'succeeded':'Prepare', 'timeout':'Prepare'})
            self.setInitialState('HalfCloseFingers')
            
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
            
            #cas d'erreur
            PreemptiveStateMachine.add('Debloque',
                      DeblocReloc(),
                      transitions={'endDeblocReloc':'problem'})
            
     
     
class BackFromMiddleObjects(PreemptiveStateMachine):
    def __init__(self):
        PreemptiveStateMachine.__init__(self,outcomes=['end','problem'])
        with self:      
            PreemptiveStateMachine.addPreemptive('EndMatchPreemption',
                                             EndMatchPreempter(-5.0),
                                             transitions={'endMatch':'end'})
            
            PreemptiveStateMachine.add('OpenFingers',
                      FingersOnlyState('open_right'), 
                      transitions={'succeeded':'SetVmax', 'timeout':'SetVmax'})
            self.setInitialState('OpenFingers')
            
            PreemptiveStateMachine.add('SetVmax', 
                      SetVMaxState(0.5),
                      transitions={'succeeded':'EngageFarBotTotem','timeout':'EngageFarBotTotem'})   
            
            PreemptiveStateMachine.add('EngageFarBotTotem',
                      AmbiOmniDirectOrder(-0.700,-0.400,0),
                      transitions={'succeeded':'Moissbat', 'timeout':'Debloque'})
            
            PreemptiveStateMachine.add('Moissbat',
                      AmbiOmniDirectOrder(0.600,-0.400,0),
                      transitions={'succeeded':'UnSetVmax', 'timeout':'Debloque'})
            
            PreemptiveStateMachine.add('UnSetVmax', 
                      SetVMaxState(0.0),
                      transitions={'succeeded':'end','timeout':'end'})   
            
         
            #cas d'erreur
            PreemptiveStateMachine.add('Debloque',
                      DeblocReloc(),
                      transitions={'endDeblocReloc':'problem'}) 