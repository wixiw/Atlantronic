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
                      AmbiOmniDirectOrder(0.250, 0.280, -pi/2),
                      transitions={'succeeded':'BeginSlashTotem', 'timeout':'Debloque'})
            
            # le C point est 13 cm devant le robot, il doit longer le bord du totem.
            PreemptiveStateMachine.add('BeginSlashTotem',
                      AmbiOmniDirectOrder_cpoint(0.090,0,0,
                                                 0.550,0.125,-pi/2),
                      transitions={'succeeded':'OpenFinger', 'timeout':'Debloque'})
            
            PreemptiveStateMachine.add('OpenFinger',
                      AmbiClawFingerOrder(Robot2012.FINGER_HALF_CLOSE,Robot2012.FINGER_OPEN,
                                          Robot2012.CLAW_HALF_CLOSE, Robot2012.CLAW_TOTEM),
                      transitions={'succeeded':'SetStratInfo_TotemFinished', 'timeout':'problem'})  
            
            PreemptiveStateMachine.add('SetStratInfo_TotemFinished',
                      SetStratInfoState('topCloseTotemFull',False),
                      transitions={'ok':'OpenClawMore'})
            
            PreemptiveStateMachine.add('OpenClawMore',
                       AmbiClawFingerOrder(Robot2012.FINGER_HALF_CLOSE,Robot2012.FINGER_OPEN,
                                           Robot2012.CLAW_HALF_CLOSE, Robot2012.CLAW_OPEN),
                      transitions={'succeeded':'EndSlashTotem', 'timeout':'problem'})  
            
            PreemptiveStateMachine.add('EndSlashTotem',
                      AmbiOmniDirectOrder_cpoint(0.060,0,0,
                                                 0.800,0.125,-pi/2),
                      transitions={'succeeded':'ThrowUp', 'timeout':'Debloque'})
             
            PreemptiveStateMachine.add('ThrowUp',
                      AmbiOmniDirectOrder(1.200,0.0,-pi/5),
                      transitions={'succeeded':'SetStratInfo_ThrowUpFinished', 'timeout':'Back'})
            
            PreemptiveStateMachine.add('SetStratInfo_ThrowUpFinished',
                      SetStratInfoState('closeFreeGoldbarInPosition', False),
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

