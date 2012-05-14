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
                      TopCloseTotem(),
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
                      transitions={'end':'ThrowUp', 'problem':'endMiddleGame'})
            
            PreemptiveStateMachine.add('ThrowUp',
                      AmbiOmniDirectOrder(1.200,0.200,pi/5),
                      transitions={'succeeded':'SetStratInfo_ThrowUpFinished', 'timeout':'Debloque'})

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
                      transitions={'endTotem':'PushABit', 'problem':'endMiddleGame'})
            
            PreemptiveStateMachine.add('PushABit',
                      AmbiOpenLoopOrder(0.300, 0.000, 0,
                                        1),
                      transitions={'succeeded':'PushBack', 'timeout':'Back'})
            
            PreemptiveStateMachine.add('PushBack',
                      AmbiOpenLoopOrder(-0.300, 0.0, 0,
                                          1),
                      transitions={'succeeded':'CloseBottleAndCoin', 'timeout':'Debloque'})
            
            
            PreemptiveStateMachine.add('CloseBottleAndCoin',
                      CloseBottleAndCoin(),
                      transitions={'end':'endMiddleGame', 'problem':'endMiddleGame'})
            
            
            
            PreemptiveStateMachine.add('Debloque',
                      Replay(1.0),
                      transitions={'succeeded':'endMiddleGame', 'timeout':'endMiddleGame'})  
