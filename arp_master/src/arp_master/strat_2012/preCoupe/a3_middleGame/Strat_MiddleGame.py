#!/usr/bin/env python

#libraries for ROS
import roslib; roslib.load_manifest('arp_master')

from arp_master import *


class MiddleGame(PreemptiveStateMachine):
    def __init__(self):
        PreemptiveStateMachine.__init__(self,outcomes=['endMiddleGame','problem'])
        with self:      
            PreemptiveStateMachine.addPreemptive('EndMatchPreemption',
                                             EndMatchPreempter(-5.0),
                                             transitions={'endMatch':'endMiddleGame'})

            PreemptiveStateMachine.add('GotoFirstBottle',
                      GotoFirstBottle(),
                      transitions={'succeeded':'GotoSecondBottle', 'timeout':'problem'})
            #as initial state is not the preemptive one, it is necessary to add the information here !
            self.setInitialState('GotoFirstBottle')
            
            PreemptiveStateMachine.add('GotoSecondBottle',
                      GotoSecondBottle(),
                      transitions={'succeeded':'endMiddleGame', 'timeout':'problem'})
        



############### Ordres de motion

class GotoFirstBottle(CyclicActionState):
    def createAction(self):
        pose = AmbiPoseRed(0.200,-0.750, -pi/2, Data.color)
        self.omnidirect(pose.x, pose.y, pose.theta)

class GotoSecondBottle(CyclicActionState):
    def createAction(self):
        pose = AmbiPoseRed(-0.600,-0.800, -pi/2, Data.color)
        self.omnidirect(pose.x, pose.y, pose.theta)
         
################# REVERSER
class ReverseOrder(CyclicActionState):
    def createAction(self):
        self.replay(1.0)  
