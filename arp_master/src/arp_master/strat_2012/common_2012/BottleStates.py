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
            
            PreemptiveStateMachine.add('Prepare',
                      AmbiOmniDirectOrder(x,y,theta),
                      transitions={'succeeded':'Push', 'timeout':'Debloque'})
            self.setInitialState('Prepare')
            
            PreemptiveStateMachine.add('Push',
                      AmbiOpenLoopOrder(0,-0.100,0,
                               2),
                      transitions={'succeeded':'SetStratInfo', 'timeout':'Debloque'})
            
            PreemptiveStateMachine.add('SetStratInfo',
                      SetStratInfoState(bottle_strat_info, True),
                      transitions={'ok':'Back'})
            
            PreemptiveStateMachine.add('Back',
                      AmbiOmniDirectOrder(x,y,theta),
                      transitions={'succeeded':'endBottle', 'timeout':'Debloque'})
            
            PreemptiveStateMachine.add('Debloque',
                      Debloque(1.0),
                      transitions={'succeeded':'problem', 'timeout':'problem'})
            
class FarBottleState(BottleState):
    def __init__(self):  
            BottleState.__init__(self,Table2012.P_BOTTLE_FAR.x-0.050,-0.700,0, "farBottlePushed")
            
            
class CloseBottleState(BottleState):
    def __init__(self):  
            BottleState.__init__(self,Table2012.P_BOTTLE_CLOSE.x-0.050,-0.700,0, "closeBottlePushed")            