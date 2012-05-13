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
                      transitions={'endTotem':'MiddleObject', 'problem':'endMiddleGame'})
            #as initial state is not the preemptive one, it is necessary to add the information here !
            self.setInitialState('TopCloseTotem')
            
            PreemptiveStateMachine.add('MiddleObject',
                      MiddleObjects(),
                      transitions={'end':'FarBottle', 'problem':'CloseBottleAndCoin'})
            
            PreemptiveStateMachine.add('FarBottle',
                      FarBottleState(),
                      transitions={'endBottle':'BackFromMiddleObjects', 'problem':'CloseBottleAndCoin'})
            
            PreemptiveStateMachine.add('BackFromMiddleObjects',
                      BackFromMiddleObjects(),
                      transitions={'end':'CloseBottleAndCoin', 'problem':'endMiddleGame'})

            PreemptiveStateMachine.add('CloseBottleAndCoin',
                      CloseBottleAndCoin(),
                      transitions={'end':'PrepareBotTotem', 'problem':'PrepareBotTotem'})
            
            PreemptiveStateMachine.add('PrepareBotTotem',
                      AmbiOmniDirectOrder(0.500, -0.500, pi/3),
                      transitions={'succeeded':'BotCloseTotem', 'timeout':'Debloque'})
            
            PreemptiveStateMachine.add('BotCloseTotem',
                      BotCloseTotem(),
                      transitions={'endTotem':'endMiddleGame', 'problem':'endMiddleGame'})
            
            PreemptiveStateMachine.add('Debloque',
                      Replay(1.0),
                      transitions={'succeeded':'endMiddleGame', 'timeout':'endMiddleGame'})  
