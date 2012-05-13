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
            
            PreemptiveStateMachine.add('Prepare',
                      AmbiOmniDirectOrder(0.875,-0.400,pi),
                      transitions={'succeeded':'EngageCoins', 'timeout':'Debloque'})
            self.setInitialState('Prepare')
            
            PreemptiveStateMachine.add('EngageCoins',
                      AmbiOmniDirectOrder(0.200,-0.800,3*pi/4),
                      transitions={'succeeded':'PassThrought', 'timeout':'Debloque'})
            
            PreemptiveStateMachine.add('PassThrought',
                      AmbiOmniDirectOrder(-0.250,-0.700,pi/4),
                      transitions={'succeeded':'SetStratInfo', 'timeout':'Debloque'})
            
            PreemptiveStateMachine.add('SetStratInfo',
                      SetStratInfoState("middleCoinsInPosition", False),
                      transitions={'ok':'end'})
            
            PreemptiveStateMachine.add('Debloque',
                      Debloque(1.0),
                      transitions={'succeeded':'problem', 'timeout':'problem'})
            
            