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
            
            PreemptiveStateMachine.add('Rotate',
                      Rotate(),
                      transitions={'succeeded':'GoDown', 'timeout':'ReverseOrder'})
            self.setInitialState('Rotate')
            
            PreemptiveStateMachine.add('GoDown',
                      GoDown(),
                      transitions={'succeeded':'GoFarBottle', 'timeout':'ReverseOrder'})
                        
            PreemptiveStateMachine.add('GoFarBottle',
                      GoFarBottle(),
                      transitions={'succeeded':'WaitBeforeNext', 'timeout':'ReverseOrder'})
            
            PreemptiveStateMachine.add('ReverseOrder',
                      ReverseOrder(),
                      transitions={'succeeded':'endOpening', 'timeout':'endOpening'})
            
            PreemptiveStateMachine.add('WaitBeforeNext',
                      WaiterState(1.0),
                      transitions={'timeout':'endOpening'})
                        
            
        
############### Ordres de motion

class Rotate(CyclicActionState):
    def createAction(self):
        self.openloop_cpoint(-0.0583,0.220,0,
                        0,0,1,
                        2.05)

class GoDown(CyclicActionState):
    def createAction(self):
        self.openloop(1.0,0,0,
                 1.5)
class GoFarBottle(CyclicActionState):
    def createAction(self):
        self.openloop(0.0,-1.0,0,
                 1.3)
   
   
################# REVERSER
class ReverseOrder(CyclicActionState):
    def createAction(self):
        self.replay(1.0)  
