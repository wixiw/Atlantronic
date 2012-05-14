#!/usr/bin/env python

#libraries for ROS
import roslib; roslib.load_manifest('arp_master')
from arp_master import *

from DynamixelActionState import *
from Table2012 import *
from Robot2012 import *

#you should not use this state, please see user states at the end of file
class CleanCloseTotem(PreemptiveStateMachine):
    def __init__(self, table_half):
        PreemptiveStateMachine.__init__(self,outcomes=['endClean','problem'])
        with self:      
            PreemptiveStateMachine.addPreemptive('EndMatchPreemption',
                                             EndMatchPreempter(-5.0),
                                             transitions={'endMatch':'endClean'})
                                       
            PreemptiveStateMachine.add('OpenRightFinger',
                      FingersOnlyState('open'), 
                      transitions={'succeeded':'GotoFirstTotemCoin', 'timeout':'GotoFirstTotemCoin'})
            #as initial state is not the preemptive one, it is necessary to add the information here !
            self.setInitialState('OpenRightFinger')
            
            pose = TotemPose(0.130, 0.370,-pi/4,table_half)
            PreemptiveStateMachine.add('GotoFirstTotemCoin',
                      AmbiOmniDirectOrder_cpoint(0.060,0.0,0.0,
                                                 pose.x, pose.y, pose.h),
                      transitions={'succeeded':'TotemClean', 'timeout':'Debloque'})
            
            pose = TotemPose(0.600,0.140,-pi/3,table_half)
            PreemptiveStateMachine.add('TotemClean',
                      AmbiOmniDirectOrder_cpoint(0.162, -0.173,0.0,
                                                pose.x, pose.y, pose.h),
                      transitions={'succeeded':'Push', 'timeout':'Debloque'})
            
            pose = TotemPose(0.650,0.000,-pi/3,table_half)
            PreemptiveStateMachine.add('Push',
                      AmbiOmniDirectOrder_cpoint(0.162, -0.173,0.0,
                                                 pose.x, pose.y, pose.h),
                      transitions={'succeeded':'Back', 'timeout':'Debloque'})
            
            pose = TotemPose(0.500,0.500,-pi/3,table_half)
            PreemptiveStateMachine.add('Back',
                      AmbiOmniDirectOrder(pose.x, pose.y, pose.h),
                      transitions={'succeeded':'CloseFingers', 'timeout':'Debloque'})
            
            PreemptiveStateMachine.add('CloseFingers',
                      FingersOnlyState('close'), 
                      transitions={'succeeded':'endClean', 'timeout':'endClean'})
            
            
            PreemptiveStateMachine.add('Debloque',
                      Replay(1.0),
                      transitions={'succeeded':'problem', 'timeout':'problem'})

#you should not use this state, please see user states at the end of file
class WorkCloseTotem(PreemptiveStateMachine):
    def __init__(self, table_half):
        PreemptiveStateMachine.__init__(self,outcomes=['endTotem','problem'])
        with self:      
            PreemptiveStateMachine.addPreemptive('EndMatchPreemption',
                                             EndMatchPreempter(-5.0),
                                             transitions={'endMatch':'endTotem'})
            PreemptiveStateMachine.add('OpenClaw',
                     ClawsOnlyState('totem_right'),
                      transitions={'succeeded':'EnterTotem', 'timeout':'EnterTotem'})  
            #as initial state is not the preemptive one, it is necessary to add the information here !
            self.setInitialState('OpenClaw')
            
            pose = TotemPose(0.250, 0.280, -pi/2,table_half)
            PreemptiveStateMachine.add('EnterTotem',
                      AmbiOmniDirectOrder(pose.x, pose.y, pose.h),
                      transitions={'succeeded':'BeginSlashTotem', 'timeout':'Debloque'})
            
            # le C point est 13 cm devant le robot, il doit longer le bord du totem.
            pose = TotemPose(0.550,0.125,-pi/2,table_half)
            PreemptiveStateMachine.add('BeginSlashTotem',
                      AmbiOmniDirectOrder_cpoint(0.090,0,0,
                                                 pose.x, pose.y, pose.h),
                      transitions={'succeeded':'OpenFinger', 'timeout':'Debloque'})
            
            PreemptiveStateMachine.add('OpenFinger',
                      AmbiClawFingerOrder(Robot2012.FINGER_HALF_CLOSE,Robot2012.FINGER_OPEN,
                                          Robot2012.CLAW_HALF_CLOSE, Robot2012.CLAW_TOTEM),
                      transitions={'succeeded':'SetStratInfo_TotemFinished', 'timeout':'SetStratInfo_TotemFinished'})  
            
            PreemptiveStateMachine.add('SetStratInfo_TotemFinished',
                      SetStratInfoState('topCloseTotemFull',False),
                      transitions={'ok':'OpenClawMore'})
            
            PreemptiveStateMachine.add('OpenClawMore',
                       AmbiClawFingerOrder(Robot2012.FINGER_HALF_CLOSE,Robot2012.FINGER_OPEN,
                                           Robot2012.CLAW_HALF_CLOSE, Robot2012.CLAW_OPEN),
                      transitions={'succeeded':'EndSlashTotem', 'timeout':'problem'})  
            
            pose = TotemPose(0.800,0.125,-pi/2,table_half)
            PreemptiveStateMachine.add('EndSlashTotem',
                      AmbiOmniDirectOrder_cpoint(0.060,0,0,
                                                 pose.x, pose.y, pose.h),
                      transitions={'succeeded':'endTotem', 'timeout':'Debloque'})
             
            PreemptiveStateMachine.add('Debloque',
                      Debloque(1.0),
                      transitions={'succeeded':'problem', 'timeout':'problem'})



#you should not use this state, please see user states at the end of file
class ThrowUpCloseTotem(PreemptiveStateMachine):
    def __init__(self, table_half):
        PreemptiveStateMachine.__init__(self,outcomes=['end','problem'])
        with self:      
            PreemptiveStateMachine.addPreemptive('EndMatchPreemption',
                                             EndMatchPreempter(-5.0),
                                             transitions={'endMatch':'end'})
            
            pose = TotemPose(1.200,0.100,-pi/5,table_half)
            PreemptiveStateMachine.add('ThrowUp',
                      AmbiOmniDirectOrder(pose.x, pose.y, pose.h),
                      transitions={'succeeded':'SetStratInfo_ThrowUpFinished', 'timeout':'Back'})
            self.setInitialState('ThrowUp')
            
            PreemptiveStateMachine.add('SetStratInfo_ThrowUpFinished',
                      SetStratInfoState('closeFreeGoldbarInPosition', False),
                      transitions={'ok':'Back'})
            
            pose = TotemPose(0.950,0.150,-pi/2,table_half)
            PreemptiveStateMachine.add('Back',
                      AmbiOmniDirectOrder(pose.x, pose.y, pose.h),
                      transitions={'succeeded':'CloseFingersAndClaws', 'timeout':'Debloque'})
            
            PreemptiveStateMachine.add('CloseFingersAndClaws',
                      FingerClawState('close'),
                      transitions={'succeeded':'end', 'timeout':'end'}) 

            PreemptiveStateMachine.add('Debloque',
                      Debloque(1.0),
                      transitions={'succeeded':'problem', 'timeout':'problem'})


#######################################################################################################
#user states 

class CleanTopCloseTotem(CleanCloseTotem):
    def __init__(self):
        CleanCloseTotem.__init__(self,"top_close")

class CleanBotCloseTotem(CleanCloseTotem):
    def __init__(self):
        CleanCloseTotem.__init__(self,"bot_close")

class TopCloseTotem(WorkCloseTotem):
    def __init__(self):
        WorkCloseTotem.__init__(self,"top_close")

class BotCloseTotem(WorkCloseTotem):
    def __init__(self):
        WorkCloseTotem.__init__(self,"bot_close")
        
class ThrowUpTopCloseTotem(ThrowUpCloseTotem):
    def __init__(self):
        ThrowUpCloseTotem.__init__(self,"top_close")

class ThrowUpBotCloseTotem(CleanCloseTotem):
    def __init__(self):
        ThrowUpCloseTotem.__init__(self,"bot_close")

#######################################################################################################
