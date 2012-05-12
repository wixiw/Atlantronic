#!/usr/bin/env python

#libraries for ROS
import roslib; roslib.load_manifest('arp_master')
from arp_master import *

from DynamixelActionState import *

class TopCloseTotem(PreemptiveStateMachine):
    def __init__(self):
        PreemptiveStateMachine.__init__(self,outcomes=['endTotem','problem'])
        with self:      
            PreemptiveStateMachine.addPreemptive('EndMatchPreemption',
                                             EndMatchPreempter(-5.0),
                                             transitions={'endMatch':'endTotem'})
            PreemptiveStateMachine.add('OpenClaw',
                      ClawFingerOrder(-1.0,-1.8,-1.8,0.0),
                      transitions={'succeeded':'EnterTotem', 'timeout':'problem'})  
            #as initial state is not the preemptive one, it is necessary to add the information here !
            self.setInitialState('OpenClaw')
            
            PreemptiveStateMachine.add('EnterTotem',
                      EnterTotem(),
                      transitions={'succeeded':'WaitBeforeSlash', 'timeout':'Debloque'})
            

            #pour avoir le temps de sauter sur l'AU pendant les tests
            PreemptiveStateMachine.add('WaitBeforeSlash',
                      WaiterState(1.0),
                      transitions={'timeout':'SlashTotem'})
            
            PreemptiveStateMachine.add('SlashTotem',
                      SlashTotem(),
                      transitions={'succeeded':'OpenClawMore', 'timeout':'Debloque'})
            
            PreemptiveStateMachine.add('OpenClawMore',
                      ClawFingerOrder(-1.8,0.5,-1.8,0.5),
                      transitions={'succeeded':'ThrowUp', 'timeout':'problem'})  
            
            PreemptiveStateMachine.add('ThrowUp',
                      ThrowUp(),
                      transitions={'succeeded':'Back', 'timeout':'Debloque'})
            
            PreemptiveStateMachine.add('Back',
                      Back(),
                      transitions={'succeeded':'CloseClaws', 'timeout':'Debloque'})
            
            PreemptiveStateMachine.add('CloseClaws',
                      ClawFingerOrder(-1.8,-1.8,-1.8,-1.8),
                      transitions={'succeeded':'endTotem', 'timeout':'problem'}) 

            PreemptiveStateMachine.add('Debloque',
                      Debloque(1.0),
                      transitions={'succeeded':'problem', 'timeout':'problem'})


############### Ordres de motion
# le C point est 12 cm devant le robot, il doit longer le bord du totem.

class EnterTotem(CyclicActionState):
    def createAction(self):
        pose = AmbiPoseRed(0.0, 0.125, -pi/2, Data.color)
        self.omnidirect_cpoint(0.130,0,0,
                               pose.x, pose.y, pose.theta)

class SlashTotem(CyclicActionState):
    def createAction(self):
        pose = AmbiPoseRed(0.900,0.125,-pi/2, Data.color)
        self.omnidirect_cpoint(0.130,0,0,
                               pose.x, pose.y, pose.theta)
        
class ThrowUp(CyclicActionState):
    def createAction(self):
        pose = AmbiPoseRed(1.250,0.0,-pi/4, Data.color)
        self.omnidirect_cpoint(0,0,0,
                               pose.x, pose.y, pose.theta)       
        
        
class Back(CyclicActionState):
    def createAction(self):
        pose = AmbiPoseRed(0.950,0.2,-pi/2, Data.color)
        self.omnidirect_cpoint(0,0,0,
                               pose.x, pose.y, pose.theta)     