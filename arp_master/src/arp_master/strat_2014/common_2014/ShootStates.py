#!/usr/bin/env python

#libraries for ROS
import roslib; roslib.load_manifest('arp_master')
from arp_master import *

from Table2014 import *
from Robot2014 import *

# This action shoot 3 balls on each mammoths (so a total of 6 balls are shot).
#In order to use this action you have to go to the entry point DoubleTargetShootState.getEntryPoint()
class DoubleTargetShootState(PreemptiveStateMachine):
    
    @staticmethod
    def getEntryYellowPose():
        return Pose2D(0.000, 0.400, -pi/2);
    
    def __init__(self):
        PreemptiveStateMachine.__init__(self,outcomes=['endShoot','problem'])
        with self:      
            PreemptiveStateMachine.addPreemptive('EndMatchPreemption',
                                             EndMatchPreempter(-Robot2014.SWITCH_TO_EOG_DELAY),
                                             transitions={'endMatch':'endShoot'})
                                       
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
                      transitions={'succeeded':'DoubleShoot_2', 'timeout':'problem'})
            
            # DoubleShoot_2            
            PreemptiveStateMachine.add('DoubleShoot_2',
                      WaiterState(0.5),
                      transitions={'timeout':'TurnDoubleShoot_3'})
            
             # Turn DoubleShoot_3
            PreemptiveStateMachine.add('TurnDoubleShoot_3',
                      AmbiTurnOrder(-pi/2 - 0.08),
                      transitions={'succeeded':'DoubleShoot_3', 'timeout':'problem'})
            
            # DoubleShoot_3           
            PreemptiveStateMachine.add('DoubleShoot_3',
                      WaiterState(0.5),
                      transitions={'timeout':'endShoot'})