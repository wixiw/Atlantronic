#!/usr/bin/env python

#libraries for ROS
import roslib; roslib.load_manifest('arp_master')
from arp_master import *

class MiddleGame(PreemptiveStateMachine):
    def __init__(self):
        PreemptiveStateMachine.__init__(self,outcomes=['endMiddleGame'])
        with self:      
            PreemptiveStateMachine.addPreemptive('EndMatchPreemption',
                                             EndMatchPreempter(-5.0),
                                             transitions={'endMatch':'endMiddleGame'})
            PreemptiveStateMachine.add('TopCloseTotem',
                      AntiWorkTopCloseTotem(),
                      transitions={'endTotem':'ThrowUpTopCloseTotem', 'problem':'endMiddleGame'})
            #as initial state is not the preemptive one, it is necessary to add the information here !
            self.setInitialState('TopCloseTotem')
            
            PreemptiveStateMachine.add('ThrowUpTopCloseTotem',
                      ThrowUpTopCloseTotem(),
                      transitions={'end':'MiddleObject', 'problem':'endMiddleGame'})
            
            
            PreemptiveStateMachine.add('MiddleObject',
                      MiddleObjects(),
                      transitions={'end':'FarBottle', 'problem':'CloseBottleAndCoin'})
            
            PreemptiveStateMachine.add('FarBottle',
                      FarBottleState(),
                      transitions={'endBottle':'BackFromMiddleObjects', 'problem':'CloseBottleAndCoin'})
            
            PreemptiveStateMachine.add('BackFromMiddleObjects',
                      BackFromMiddleObjects(),
                      transitions={'end':'ThrowUp', 'problem':'CloseBottleAndCoin'})
            
            PreemptiveStateMachine.add('ThrowUp',
                      AmbiOmniDirectOrder(1.150,0.150,pi/5),
                      transitions={'succeeded':'SetStratInfo_ThrowUpFinished', 'timeout':'Back'})

            PreemptiveStateMachine.add('SetStratInfo_ThrowUpFinished',
                      SetStratInfoState('closeFreeGoldbarInPosition', False),
                      transitions={'ok':'Back'})
    
            PreemptiveStateMachine.add('Back',
                      AmbiOmniDirectOrder(0.950,-0.100,pi/2),
                      transitions={'succeeded':'CloseFingersAndClaws', 'timeout':'Debloque'})
            
            PreemptiveStateMachine.add('CloseFingersAndClaws',
                      FingerClawState('close'),
                      transitions={'succeeded':'PrepareBotTotem', 'timeout':'CloseBottleAndCoin'}) 

            PreemptiveStateMachine.add('PrepareBotTotem',
                      AmbiOmniDirectOrder(0.500, -0.500, pi/3),
                      transitions={'succeeded':'BotCloseTotem', 'timeout':'Debloque'})
            
            PreemptiveStateMachine.add('BotCloseTotem',
                      BotCloseTotem(),
                      transitions={'endTotem':'CloseBottleAndCoin', 'problem':'CloseBottleAndCoin'})
        
            PreemptiveStateMachine.add('CloseBottleAndCoin',
                      CloseBottleAndCoin(),
                      transitions={'end':'endMiddleGame', 'problem':'endMiddleGame'})
            
            PreemptiveStateMachine.add('Debloque',
                      Replay(1.0),
                      transitions={'succeeded':'endMiddleGame', 'timeout':'endMiddleGame'})  
