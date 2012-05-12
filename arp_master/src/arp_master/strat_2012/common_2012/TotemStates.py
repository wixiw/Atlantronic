#!/usr/bin/env python

#libraries for ROS
import roslib; roslib.load_manifest('arp_master')
from arp_master import *

from DynamixelActionState import *
from Table2012 import *
from Robot2012 import *

class TopCloseTotem(PreemptiveStateMachine):
    def __init__(self):
        PreemptiveStateMachine.__init__(self,outcomes=['endTotem','problem'])
        with self:      
            PreemptiveStateMachine.addPreemptive('EndMatchPreemption',
                                             EndMatchPreempter(-5.0),
                                             transitions={'endMatch':'endTotem'})
            PreemptiveStateMachine.add('OpenClaw',
                     ClawsOnlyState('totem_right'),
                      transitions={'succeeded':'EnterTotem', 'timeout':'problem'})  
            #as initial state is not the preemptive one, it is necessary to add the information here !
            self.setInitialState('OpenClaw')
            
            PreemptiveStateMachine.add('EnterTotem',
                      AmbiOmniDirectOrder_cpoint(0.130,0,0,
                                                 0.0, 0.125, -pi/2),
                      transitions={'succeeded':'WaitBeforeSlash', 'timeout':'Debloque'})
            

            #pour avoir le temps de sauter sur l'AU pendant les tests
            PreemptiveStateMachine.add('WaitBeforeSlash',
                      WaiterState(1.0),
                      transitions={'timeout':'SlashTotem'})
            
            # le C point est 13 cm devant le robot, il doit longer le bord du totem.
            PreemptiveStateMachine.add('SlashTotem',
                      AmbiOmniDirectOrder_cpoint(0.130,0,0,
                                                 0.900,0.125,-pi/2),
                      transitions={'succeeded':'SetStratInfo_TotemFinished', 'timeout':'Debloque'})
            
            PreemptiveStateMachine.add('SetStratInfo_TotemFinished',
                      SetStratInfo_TotemFinished(),
                      transitions={'ok':'OpenClawMore'})
            
            PreemptiveStateMachine.add('OpenClawMore',
                      AmbiClawFingerOrder(-1.8,0.5,-1.8,0.5),
                      transitions={'succeeded':'ThrowUp', 'timeout':'problem'})  
             
            PreemptiveStateMachine.add('ThrowUp',
                      AmbiOmniDirectOrder(1.250,0.0,-pi/4),
                      transitions={'succeeded':'SetStratInfo_ThrowUpFinished', 'timeout':'Debloque'})
            
            PreemptiveStateMachine.add('SetStratInfo_ThrowUpFinished',
                      SetStratInfo_ThrowUpFinished(),
                      transitions={'ok':'Back'})
            
            PreemptiveStateMachine.add('Back',
                      AmbiOmniDirectOrder(0.950,0.2,-pi/2),
                      transitions={'succeeded':'CloseClaws', 'timeout':'Debloque'})
            
            PreemptiveStateMachine.add('CloseClaws',
                      FingerClawState('close'),
                      transitions={'succeeded':'endTotem', 'timeout':'problem'}) 

            PreemptiveStateMachine.add('Debloque',
                      Debloque(1.0),
                      transitions={'succeeded':'problem', 'timeout':'problem'})

   
class SetStratInfo_TotemFinished(smach.State):
    def __init__(self):
        smach.State.__init__(self,['ok'])
    def execute(self,userdata):
        Table2012.topCloseTotemFull = False
        return 'ok'
        
class SetStratInfo_ThrowUpFinished(smach.State):
    def __init__(self):
        smach.State.__init__(self,['ok'])
    def execute(self,userdata):
        Table2012.closeFreeGoldbarInPosition = False
        return 'ok'
    