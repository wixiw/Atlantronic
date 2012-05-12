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
                      transitions={'succeeded':'endOpening', 'timeout':'Debloque'})
            self.setInitialState('GotoMilieu')
            
            PreemptiveStateMachine.add('Debloque',
                      Debloque(1.0),
                      transitions={'succeeded':'problem', 'timeout':'problem'})
            
        
############### Ordres de motion

class GotoMilieu(CyclicActionState):
    def createAction(self):
        pose = AmbiPoseRed(0.000, 0.700,0, Data.color)
        self.omnidirect(pose.x, pose.y, pose.theta)

         
