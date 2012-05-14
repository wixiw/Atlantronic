#!/usr/bin/env python

#libraries for ROS
import roslib; roslib.load_manifest('arp_master')
from arp_master import *

from DynamixelActionState import *
from Table2012 import *
from Robot2012 import *

class BottleState(PreemptiveStateMachine):
    def __init__(self,x,y,theta,bottle_strat_info):
        PreemptiveStateMachine.__init__(self,outcomes=['endBottle','problem'])
        with self:      
            PreemptiveStateMachine.addPreemptive('EndMatchPreemption',
                                             EndMatchPreempter(-5.0),
                                             transitions={'endMatch':'endBottle'})
            
            #onveut taper avec le milieu de la tourelle droite d'ou le -0.058m pour tenir compte d'omni order qui pilote le CdG
            PreemptiveStateMachine.add('Prepare',
                      AmbiOmniDirectOrder(x-0.058,y,theta),
                      transitions={'succeeded':'Push', 'timeout':'Debloque'})
            self.setInitialState('Prepare')
            
            PreemptiveStateMachine.add('Push',
                      AmbiOpenLoopOrder(0,-0.200,0,
                               2),
                      transitions={'succeeded':'SetStratInfo', 'timeout':'SetStratInfo'})
            
            PreemptiveStateMachine.add('SetStratInfo',
                      SetStratInfoState(bottle_strat_info, True),
                      transitions={'ok':'Back'})
            
            PreemptiveStateMachine.add('Back',
                      AmbiOmniDirectOrder(x-0.058,y,theta),
                      transitions={'succeeded':'endBottle', 'timeout':'Debloque'})
            
            PreemptiveStateMachine.add('Debloque',
                      Replay(1.0),
                      transitions={'succeeded':'problem', 'timeout':'problem'})
            
            
class FarBottleState(BottleState):
    def __init__(self):  
            BottleState.__init__(self,Table2012.P_BOTTLE_FAR.x,-0.700,0, "farBottlePushed")
        
#pour la close bottle on a besoin de se rapprocher de notre zone de chiage pour recuperer le CD.            
class CloseBottleState(BottleState):
    def __init__(self):  
            BottleState.__init__(self,Table2012.P_BOTTLE_CLOSE.x+0.058,-0.700,0, "closeBottlePushed")   
            
            
            
class CloseBottleAndCoin(PreemptiveStateMachine):
    def __init__(self):
        PreemptiveStateMachine.__init__(self,outcomes=['end','problem'])
        with self:      
            PreemptiveStateMachine.addPreemptive('EndMatchPreemption',
                                             EndMatchPreempter(-5.0),
                                             transitions={'endMatch':'end'})            
            
            PreemptiveStateMachine.add('CloseBottle',
                      CloseBottleState(),
                      transitions={'endBottle':'ThrowUp1', 'problem':'problem'})
            self.setInitialState('CloseBottle')
            
            #get close bot coin and throw up
            PreemptiveStateMachine.add('ThrowUp1',
                      AmbiOmniDirectOrder(0.850,-0.200, pi/3),
                      transitions={'succeeded':'SetStratInfoCoin', 'timeout':'Debloque'})
            
            PreemptiveStateMachine.add('SetStratInfoCoin',
                      SetStratInfoState("botCloseCoinInPosition", False),
                      transitions={'ok':'ThrowUp2'})
            
            PreemptiveStateMachine.add('ThrowUp2',
                      AmbiOmniDirectOrder(1.100,0.200,0),
                      transitions={'succeeded':'SetStratInfoGoldBar', 'timeout':'Debloque'})
            
            PreemptiveStateMachine.add('SetStratInfoGoldBar',
                      SetStratInfoState("botCloseCoinInPosition", False),
                      transitions={'ok':'end'})       
            
            PreemptiveStateMachine.add('Debloque',
                      Replay(1.0),
                      transitions={'succeeded':'problem', 'timeout':'problem'})  