#!/usr/bin/env python

#libraries for ROS
import roslib; roslib.load_manifest('arp_master')

from arp_master import *

class MiddleGame(PreemptiveStateMachine):
    def __init__(self):
        PreemptiveStateMachine.__init__(self,outcomes=['endMiddleGame'])
        with self:      
            PreemptiveStateMachine.addPreemptive('EndMatchPreemption',
                                             EndMatchPreempter(-5.0),
                                             transitions={'endMatch':'endMiddleGame'})
            # other states
            PreemptiveStateMachine.add('EtatA',
                      EtatA(),
                      transitions={'succeeded':'EtatB', 'aborted':'ReverseOrder'})
            #as initial state is not the preemptive one, it is necessary to add the information here !
            self.setInitialState('EtatA')
            
            PreemptiveStateMachine.add('EtatB',
                      EtatB(),
                      transitions={'succeeded':'EtatA', 'aborted':'ReverseOrder'})

            PreemptiveStateMachine.add('ReverseOrder',
                      ReverseOrder(),
                      transitions={'succeeded':'endMiddleGame', 'aborted':'endMiddleGame'})


############### Ordres de motion

class EtatA(CyclicActionState):
    def createAction(self):
        pose = AmbiPoseRed(0.7, 0.7,0, Data.color)
        self.pointcap(pose.x, pose.y, pose.theta)

class EtatB(CyclicActionState):
    def createAction(self):
        pose = AmbiPoseRed(-0.7,-0.7,pi, Data.color)
        self.pointcap(pose.x, pose.y, pose.theta)
         
################# REVERSER
class ReverseOrder(CyclicActionState):
    def createAction(self):
        order=Data.listReplayOrders.pop() #retourne le dernier element et l'enleve de la liste
        self.executeReplayOrder(order) 
