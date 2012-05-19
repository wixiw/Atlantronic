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
                                             EndMatchPreempter(-Robot2012.END_GAME_DELAY),
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
                      transitions={'succeeded':'PassThrought', 'timeout':'RetryPushBack'})
            
            #si on a tape le mur on essaye plus loin 
            PreemptiveStateMachine.add('RetryPushBack',
                      AmbiOmniDirectOrder(0.250,-0.700,3*pi/4),
                      transitions={'succeeded':'RetryPush', 'timeout':'RetryPush'})
            PreemptiveStateMachine.add('RetryPush',
                      AmbiOmniDirectOrder(-0.100,-0.700,3*pi/4),
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
                                             EndMatchPreempter(-Robot2012.END_GAME_DELAY),
                                             transitions={'endMatch':'end'})
            
            
            
            PreemptiveStateMachine.add('EngageFarBotTotem',
                      AmbiOmniDirectOrder(-0.700,-0.420,0),
                      transitions={'succeeded':'Moissbat', 'timeout':'Debloque'})
            self.setInitialState('OpenFingers')
            
            PreemptiveStateMachine.add('OpenFingers',
                      FingersOnlyState('open_right'), 
                      transitions={'succeeded':'EngageFarBotTotem', 'timeout':'EngageFarBotTotem'})
            
            PreemptiveStateMachine.add('Moissbat',
                      AmbiOmniDirectOrder(0.550,-0.420,0, 0.4),
                      transitions={'succeeded':'end', 'timeout':'Debloque'})
            
         
            #cas d'erreur
            PreemptiveStateMachine.add('Debloque',
                      DeblocReloc(),
                      transitions={'endDeblocReloc':'problem'}) 
            
            
class BackFromHalfMiddleObjects(PreemptiveStateMachine):
    def __init__(self):
        PreemptiveStateMachine.__init__(self,outcomes=['end','problem'])
        with self:      
            PreemptiveStateMachine.addPreemptive('EndMatchPreemption',
                                             EndMatchPreempter(-Robot2012.END_GAME_DELAY),
                                             transitions={'endMatch':'end'})
            
            
            
            PreemptiveStateMachine.add('EngageFarBotTotem',
                      AmbiOmniDirectOrder(-0.300,-0.430,pi/5),
                      transitions={'succeeded':'Moissbat', 'timeout':'Debloque'})
            self.setInitialState('OpenFingers')
            
            PreemptiveStateMachine.add('OpenFingers',
                      FingersOnlyState('open_right'), 
                      transitions={'succeeded':'EngageFarBotTotem', 'timeout':'EngageFarBotTotem'})
            
            PreemptiveStateMachine.add('Moissbat',
                      AmbiOmniDirectOrder(0.550,-0.430,pi/5, 0.4),
                      transitions={'succeeded':'end', 'timeout':'Debloque'})
            
         
            #cas d'erreur
            PreemptiveStateMachine.add('Debloque',
                      DeblocReloc(),
                      transitions={'endDeblocReloc':'problem'}) 