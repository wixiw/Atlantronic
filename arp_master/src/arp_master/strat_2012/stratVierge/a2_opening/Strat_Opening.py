#!/usr/bin/env python

#libraries for ROS
import roslib; roslib.load_manifest('arp_master')

from arp_master import *

class Opening(PreemptiveStateMachine):
    def __init__(self):
        PreemptiveStateMachine.__init__(self, outcomes=['endOpening', 'problem'])
        with self:
            PreemptiveStateMachine.addPreemptive('EndMatchPreemption',
                                             EndMatchPreempter(-5.0),
                                             transitions={'endMatch':'problem'})
            
            PreemptiveStateMachine.add('GotoMilieu',
                      GotoMilieu(),
                      transitions={'succeeded':'WaitBeforeNext', 'timeout':'ReverseOrder'})
            
            self.setInitialState('GotoMilieu')
            
            PreemptiveStateMachine.add('ReverseOrder',
                      ReverseOrder(),
                      transitions={'succeeded':'GotoMilieu', 'timeout':'GotoMilieu'})
            
            PreemptiveStateMachine.add('WaitBeforeNext',
                      WaiterState(1.0),
                      transitions={'timeout':'endOpening'})
                        
            
        
############### Ordres de motion

class GotoMilieu(CyclicActionState):
    def createAction(self):
        pose = AmbiPoseRed(-0.500, Table.HWALL_Y-1.000,0, Data.color)
        self.pointcap(pose.x, pose.y, pose.theta)

         
################# REVERSER
class ReverseOrder(CyclicActionState):
    def createAction(self):
        self.replay(1.0)  
