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
                                       
            PreemptiveStateMachine.add('OpenFinger',
                      FingersOnlyState('open'), 
                      transitions={'succeeded':'GotoFirstTotemCoin', 'timeout':'GotoFirstTotemCoin'})
            #as initial state is not the preemptive one, it is necessary to add the information here !
            self.setInitialState('OpenFinger')
            
            pose = TotemPose(0.080, 0.450,-pi/4,table_half)
            PreemptiveStateMachine.add('GotoFirstTotemCoin',
                      AmbiOmniDirectOrder(pose.x, pose.y, pose.h),
                      transitions={'succeeded':'TotemClean', 'timeout':'Debloque'})
            
            pose = TotemPose(0.600,0.400,-pi/4,table_half)
            PreemptiveStateMachine.add('TotemClean',
                      AmbiOmniDirectOrder(pose.x, pose.y, pose.h),
                      transitions={'succeeded':'PushALot', 'timeout':'Back'})
            
            pose = TotemPose(1.10,0.150,0,table_half)
            PreemptiveStateMachine.add('PushALot',
                      AmbiOmniDirectOrder(pose.x, pose.y, pose.h),
                      transitions={'succeeded':'Back', 'timeout':'Back'})
            
            pose = TotemPose(0.700,0.500,-pi/2,table_half)
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
            pose = TotemPose(0.350,0.400,-pi/4,table_half)
            PreemptiveStateMachine.add('ApproachTotem',
                      AmbiOmniDirectOrder(pose.x, pose.y, pose.h),
                      transitions={'succeeded':'OpenClaws', 'timeout':'Debloque'})

            #as initial state is not the preemptive one, it is necessary to add the information here !
            self.setInitialState('ApproachTotem')
            
            PreemptiveStateMachine.add('OpenClaws',
                      AmbiClawFingerOrder(Robot2012.FINGER_HALF_CLOSE,Robot2012.FINGER_HALF_CLOSE,
                                          Robot2012.CLAW_HALF_CLOSE, Robot2012.CLAW_TOTEM),
                      transitions={'succeeded':'EndSlashTotem', 'timeout':'EndSlashTotem'})  
            
            pose = TotemPose(0.200, 0.260, -pi/2,table_half)
            PreemptiveStateMachine.add('EnterTotem',
                      AmbiOmniDirectOrder(pose.x, pose.y, pose.h),
                      transitions={'succeeded':'OpenFinger', 'timeout':'Debloque'})
            #as initial state is not the preemptive one, it is necessary to add the information here !
            self.setInitialState('EnterTotem')
            
            
            PreemptiveStateMachine.add('OpenFinger',
                      AmbiClawFingerOrder(Robot2012.FINGER_HALF_CLOSE,Robot2012.FINGER_HALF_CLOSE,
                                          Robot2012.CLAW_HALF_CLOSE, Robot2012.CLAW_TOTEM),
                      transitions={'succeeded':'EndSlashTotem', 'timeout':'EndSlashTotem'})  
            
            pose = TotemPose(0.750,0.125,-pi/2,table_half)
            PreemptiveStateMachine.add('EndSlashTotem',
                      AmbiOmniDirectOrder_cpoint(0.060,0,0,
                                                 pose.x, pose.y, pose.h),
                      transitions={'succeeded':'SetStratInfo_TotemFinished', 'timeout':'Debloque'})
            
            PreemptiveStateMachine.add('SetStratInfo_TotemFinished',
                      SetStratInfoState('topCloseTotemFull',False),
                      transitions={'ok':'PushABit'})
            
            pose = TotemPose(0.750,0.060,-pi/2,table_half)
            PreemptiveStateMachine.add('PushABit',
                      AmbiOmniDirectOrder_cpoint(0.060,0,0,
                                                 pose.x, pose.y, pose.h),
                      transitions={'succeeded':'endTotem', 'timeout':'Debloque'})
             
            PreemptiveStateMachine.add('Debloque',
                      Replay(1.0),
                      transitions={'succeeded':'problem', 'timeout':'problem'})


#you should not use this state, please see user states at the end of file
class AntiWorkCloseTotem(PreemptiveStateMachine):
    def __init__(self, table_half):
        PreemptiveStateMachine.__init__(self,outcomes=['endTotem','problem'])
        with self:      
            PreemptiveStateMachine.addPreemptive('EndMatchPreemption',
                                             EndMatchPreempter(-5.0),
                                             transitions={'endMatch':'endTotem'})

            
            pose = TotemPose(0.100, 0.280, -pi/2,table_half)
            PreemptiveStateMachine.add('EnterTotem',
                      AmbiOmniDirectOrder(pose.x, pose.y, pose.h),
                      transitions={'succeeded':'OpenClaws', 'timeout':'Debloque'})
            #as initial state is not the preemptive one, it is necessary to add the information here !
            self.setInitialState('EnterTotem')
                        
            PreemptiveStateMachine.add('OpenClaws',
                      AmbiClawFingerOrder(Robot2012.FINGER_CLOSE,Robot2012.FINGER_CLOSE,
                                          Robot2012.CLAW_OPEN, Robot2012.CLAW_TOTEM),
                      transitions={'succeeded':'SlashTotem', 'timeout':'SlashTotem'})  
            
            pose = TotemPose(0.500,0.125,-pi/2,table_half)
            PreemptiveStateMachine.add('SlashTotem',
                      AmbiOmniDirectOrder_cpoint(0.100,0,0,
                                                 pose.x, pose.y, pose.h),
                      transitions={'succeeded':'SetStratInfo_TotemFinished', 'timeout':'Debloque'})
            
            PreemptiveStateMachine.add('SetStratInfo_TotemFinished',
                      SetStratInfoState('topCloseTotemFull',False),
                      transitions={'ok':'OpenAll'})
            
            PreemptiveStateMachine.add('OpenAll',
                      FingerClawState("open"),
                      transitions={'succeeded':'EndSlashTotem', 'timeout':'EndSlashTotem'})  
            
            pose = TotemPose(0.750,0.125,-pi/2,table_half)
            PreemptiveStateMachine.add('EndSlashTotem',
                      AmbiOmniDirectOrder_cpoint(0.100,0,0,
                                                 pose.x, pose.y, pose.h),
                      transitions={'succeeded':'PushABit', 'timeout':'Debloque'})
            
            pose = TotemPose(0.750,0.000,-pi/2,table_half)
            PreemptiveStateMachine.add('PushABit',
                      AmbiOmniDirectOrder_cpoint(0.060,0,0,
                                                 pose.x, pose.y, pose.h),
                      transitions={'succeeded':'endTotem', 'timeout':'Debloque'})
             
            PreemptiveStateMachine.add('Debloque',
                      Replay(1.0),
                      transitions={'succeeded':'problem', 'timeout':'problem'})



#you should not use this state, please see user states at the end of file
class ThrowUpCloseTotem(PreemptiveStateMachine):
    def __init__(self, table_half):
        PreemptiveStateMachine.__init__(self,outcomes=['end','problem'])
        with self:      
            PreemptiveStateMachine.addPreemptive('EndMatchPreemption',
                                             EndMatchPreempter(-5.0),
                                             transitions={'endMatch':'end'})
            
                                    
            PreemptiveStateMachine.add('OpenClawMore',
                       AmbiClawFingerOrder(Robot2012.FINGER_HALF_CLOSE,Robot2012.FINGER_OPEN,
                                           Robot2012.CLAW_HALF_CLOSE, Robot2012.CLAW_OPEN),
                      transitions={'succeeded':'ThrowUp', 'timeout':'ThrowUp'})  
            self.setInitialState('OpenClawMore')
            
            pose = TotemPose(1.180,0.080,-pi/5,table_half)
            PreemptiveStateMachine.add('ThrowUp',
                      AmbiOmniDirectOrder(pose.x, pose.y, pose.h),
                      transitions={'succeeded':'SetStratInfo_ThrowUpFinished', 'timeout':'Back'})
            
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
                      Replay(1.0),
                      transitions={'succeeded':'problem', 'timeout':'problem'})


#######################################################################################################
#user states 

class CleanTopCloseTotem(CleanCloseTotem):
    def __init__(self):
        CleanCloseTotem.__init__(self,"top_close")

class CleanBotCloseTotem(CleanCloseTotem):
    def __init__(self):
        CleanCloseTotem.__init__(self,"bot_close")

#avec la griffe droite
class TopCloseTotem(WorkCloseTotem):
    def __init__(self):
        WorkCloseTotem.__init__(self,"top_close")

#avec la griffe gauche
class BotCloseTotem(WorkCloseTotem):
    def __init__(self):
        WorkCloseTotem.__init__(self,"bot_close")

#avec la griffe gauche
class AntiWorkTopCloseTotem(AntiWorkCloseTotem):
    def __init__(self):
        AntiWorkCloseTotem.__init__(self,"top_close")

#avec la griffe droite
class AntiWorkBotCloseTotem(AntiWorkCloseTotem):
    def __init__(self):
        AntiWorkCloseTotem.__init__(self,"bot_close")
                
class ThrowUpTopCloseTotem(ThrowUpCloseTotem):
    def __init__(self):
        ThrowUpCloseTotem.__init__(self,"top_close")

class ThrowUpBotCloseTotem(CleanCloseTotem):
    def __init__(self):
        ThrowUpCloseTotem.__init__(self,"bot_close")

#######################################################################################################
