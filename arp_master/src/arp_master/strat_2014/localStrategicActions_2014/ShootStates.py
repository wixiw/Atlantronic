#!/usr/bin/env python
# -*- coding: latin-1 -*-

#libraries for ROS
import roslib; roslib.load_manifest('arp_master')
from arp_master import *

from arp_master.util import *
from arp_master.fsmFramework import *
from arp_master.commonStates import *
from arp_master.strat_2014.common_2014 import *

# This action shoot 3 balls on each mammoths (so a total of 6 balls are shot).
#In order to use this action you have to go to the entry point DoubleTargetShootState.getEntryPoint()
class DoubleTargetShootState(LocalStrategicAction):
    
    def getEntryYellowPose(self):
        return Pose2D(0.000, 0.400, -pi/2);
    
    def __init__(self):
        LocalStrategicAction.__init__(self, Robot2014.SWITCH_TO_EOG_DELAY)
        with self:      
            # DoubleShoot : mock state as Autom V2 doesn't allow pure rotation, waiting bugfixes to uncomment below (done)
            PreemptiveStateMachine.add('DoubleShoot',
                      WaiterState(0.5),
                      transitions={'timeout':'TurnDoubleShoot_2'})
                      #transitions={'timeout':'endShoot'})
            self.setInitialState('DoubleShoot')
            
# 2x DoubleShoots on Mammoths, turn of 0.085 RAD = 5 DEG

             # Turn DoubleShoot_2
            PreemptiveStateMachine.add('TurnDoubleShoot_2',
                      AmbiTurnOrder(-pi/2 + 0.00),
                      transitions={'succeeded':'DoubleShoot_2', 'timeout':'DoubleShoot_2'})
            
            # DoubleShoot_2            
            PreemptiveStateMachine.add('DoubleShoot_2',
                      WaiterState(0.5),
                      transitions={'timeout':'TurnDoubleShoot_3'})
            
             # Turn DoubleShoot_3
            PreemptiveStateMachine.add('TurnDoubleShoot_3',
                      AmbiTurnOrder(-pi/2 - 0.08),
                      transitions={'succeeded':'DoubleShoot_3', 'timeout':'DoubleShoot_3'})
            
            # DoubleShoot_3           
            PreemptiveStateMachine.add('DoubleShoot_3',
                      WaiterState(0.5),
                      transitions={'timeout':'succeeded'})
            
            
            