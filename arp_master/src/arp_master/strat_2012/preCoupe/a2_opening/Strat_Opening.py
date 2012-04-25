#!/usr/bin/env python

#libraries for ROS
import roslib; roslib.load_manifest('arp_master')

from arp_master import *
import os

class Opening(PreemptiveStateMachine):
    def __init__(self):
        PreemptiveStateMachine.__init__(self, outcomes=['endOpening', 'problem'])
        with self:
            PreemptiveStateMachine.addPreemptive('EndMatchPreemption',
                                             EndMatchPreempter(-5.0),
                                             transitions={'endMatch':'problem'})
            
            PreemptiveStateMachine.add('GotoOutStartPos',
                      GotoOutStartPos(),
                      transitions={'succeeded':'TurnToGoldBar', 'timeout':'problem'})
            
            self.setInitialState('GotoOutStartPos')
            
            PreemptiveStateMachine.add('TurnToGoldBar',
                      TurnToGoldBar(),
                      transitions={'succeeded':'GoToGoldBar', 'timeout':'problem'})
            
            PreemptiveStateMachine.add('GoToGoldBar',
                      GoToGoldBar(),
                      transitions={'succeeded':'PushGoldBar', 'timeout':'problem'})
            
            PreemptiveStateMachine.add('PushGoldBar',
                      PushGoldBar(),
                      transitions={'succeeded':'endOpening', 'timeout':'problem'})
                        
            
        
############### Ordres de motion


## oui oui on peut faire plus court et supprimer la rotation pure, mais c'est pour tester des cas et commencer par le plus simple !
class GotoOutStartPos(CyclicActionState):
    def createAction(self):
        pose = AmbiPoseRed(0.750,0.700, -pi, Data.color)
        self.omnidirect(pose.x, pose.y, pose.theta)


class TurnToGoldBar(CyclicActionState):
    def createAction(self):
        self.omnicap(AmbiCapRed(-pi/2, Data.color).angle)
        
        
class GoToGoldBar(CyclicActionState):
    def createAction(self):
        pose = AmbiPoseRed(0.900,0, 0, Data.color)
        self.omnidirect(pose.x, pose.y, pose.theta)

class PushGoldBar(CyclicActionState):
    def createAction(self):
        pose = AmbiPoseRed(1.200,0, 0, Data.color)
        self.omnidirect(pose.x, pose.y, pose.theta)
    
    def executeOut(self):
        os.system("beep -f 300 -l100 -r1") 
