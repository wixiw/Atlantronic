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
                      transitions={'end':'BottleFar', 'problem':'BottleClose'})
            #as initial state is not the preemptive one, it is necessary to add the information here !
            
            PreemptiveStateMachine.add('BottleFar',
                      FarBottleState(),
                      transitions={'endBottle':'BottleClose', 'problem':'BottleClose'})
            
            PreemptiveStateMachine.add('BottleClose',
                      CloseBottleState(),
                      transitions={'endBottle':'ThrowUp1', 'problem':'endMiddleGame'})
            
            
            #get close bot coin and throw up
            
            PreemptiveStateMachine.add('ThrowUp1',
                      AmbiOmniDirectOrder(0.850,-0.200, pi/3),
                      transitions={'succeeded':'SetStratInfoCoin', 'timeout':'endMiddleGame'})
            
            PreemptiveStateMachine.add('SetStratInfoCoin',
                      SetStratInfoState("botCloseCoinInPosition", False),
                      transitions={'ok':'ThrowUp2'})
            
            PreemptiveStateMachine.add('ThrowUp2',
                      AmbiOmniDirectOrder(1.100,0.200,0),
                      transitions={'succeeded':'SetStratInfoGoldBar', 'timeout':'endMiddleGame'})
            
            PreemptiveStateMachine.add('SetStratInfoGoldBar',
                      SetStratInfoState("botCloseCoinInPosition", False),
                      transitions={'ok':'endMiddleGame'})
            
