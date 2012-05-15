#!/usr/bin/env python

#libraries for ROS
import roslib; roslib.load_manifest('arp_master')
from arp_master import *

from DynamixelActionState import *
from DeblocReloc import *
from Table2012 import *
from Robot2012 import *

#you should not use this state, please see user states at the end of file
# TODO : necessite une review, code non utilise et tripote.
class CleanCloseTotem(PreemptiveStateMachine):
    def __init__(self, table_half):
        PreemptiveStateMachine.__init__(self,outcomes=['endClean','problem'])
        with self:      
            PreemptiveStateMachine.addPreemptive('EndMatchPreemption',
                                             EndMatchPreempter(-5.0),
                                             transitions={'endMatch':'endClean'})
                                       
            PreemptiveStateMachine.add('OpenFinger',
                      FingersOnlyState('open'), 
                      transitions={'succeeded':'SetVmax', 'timeout':'GotoFirstTotemCoin'})
            #as initial state is not the preemptive one, it is necessary to add the information here !
            self.setInitialState('OpenFinger')
            
            PreemptiveStateMachine.add('SetVmax', 
                      SetVMaxState(0.3),
                      transitions={'succeeded':'GotoFirstTotemCoin','timeout':'GotoFirstTotemCoin'})   
            
            poseFirst = TotemPose(0.080, 0.450,-pi/4,table_half)
            PreemptiveStateMachine.add('GotoFirstTotemCoin',
                      AmbiOmniDirectOrder(poseFirst.x, poseFirst.y, poseFirst.h),
                      transitions={'succeeded':'UnSetVmax', 'timeout':'Debloque'})
            
            PreemptiveStateMachine.add('UnSetVmax', 
                      SetVMaxState(0.0),
                      transitions={'succeeded':'TotemClean','timeout':'TotemClean'})   
            
            poseClean = TotemPose(0.600,0.420,-pi/4,table_half)
            PreemptiveStateMachine.add('TotemClean',
                      AmbiOmniDirectOrder(poseClean.x, poseClean.y, poseClean.h),
                      transitions={'succeeded':'PushALot', 'timeout':'Retry'})
            
            pose = TotemPose(1.10,0.150,pi/12,table_half)
            PreemptiveStateMachine.add('PushALot',
                      AmbiOmniDirectOrder(pose.x, pose.y, pose.h),
                      transitions={'succeeded':'CloseFingersABit', 'timeout':'Back'})
            
            PreemptiveStateMachine.add('CloseFingersABit',
                      TotemClawState(0.25,Robot2012.FINGER_OPEN,
                                          Robot2012.CLAW_CLOSE, Robot2012.CLAW_CLOSE, table_half), 
                      transitions={'succeeded':'Back', 'timeout':'Back'})
            
            pose = TotemPose(0.700,0.400,-pi/2,table_half)
            PreemptiveStateMachine.add('Back',
                      AmbiOmniDirectOrder(pose.x, pose.y, pose.h),
                      transitions={'succeeded':'CloseFingers', 'timeout':'Debloque'})
            
            PreemptiveStateMachine.add('CloseFingers',
                      FingersOnlyState('close'), 
                      transitions={'succeeded':'endClean', 'timeout':'endClean'})
            
            #si on a rate le nettoyage le doigt droit s'est peut etre pris dans le totem
            #on recommence une fois en partant de plus loin   
            PreemptiveStateMachine.add('Retry',
                      Replay(1.0),
                      transitions={'succeeded':'RetryFirstTotemCoin', 'timeout':'ExitState'})
            PreemptiveStateMachine.add('RetryFirstTotemCoin',
                      AmbiOmniDirectOrder(poseFirst.x, poseFirst.y + 0.030, poseFirst.h, 0.3),
                      transitions={'succeeded':'RetryTotemClean', 'timeout':'ExitState'})
            PreemptiveStateMachine.add('RetryTotemClean',
                      AmbiOmniDirectOrder(poseClean.x, poseClean.y + 0.030, poseClean.h),
                      transitions={'succeeded':'PushALot', 'timeout':'ExitState'})
            
                        
            #cas d'erreur
            PreemptiveStateMachine.add('Debloque',
                      DeblocReloc(),
                      transitions={'endDeblocReloc':'problem'}) 
            
            PreemptiveStateMachine.add('ExitState',
                      WaiterState(0.0), 
                      transitions={'timeout':'problem'}) 


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
                      TotemClawState(Robot2012.FINGER_CLOSE,Robot2012.FINGER_CLOSE,
                                          Robot2012.CLAW_CLOSE, Robot2012.CLAW_TOTEM, table_half),
                      transitions={'succeeded':'EnterTotem', 'timeout':'EndSlashTotem'})  
            
            pose = TotemPose(0.200, 0.260, -pi/2,table_half)
            PreemptiveStateMachine.add('EnterTotem',
                      AmbiOmniDirectOrder(pose.x, pose.y, pose.h),
                      transitions={'succeeded':'EndSlashTotem', 'timeout':'Debloque'})
              
            
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
             
            #cas d'erreur
            PreemptiveStateMachine.add('Debloque',
                      DeblocReloc(),
                      transitions={'endDeblocReloc':'problem'})


#you should not use this state, please see user states at the end of file
class AntiWorkCloseTotem(PreemptiveStateMachine):
    def __init__(self, table_half):
        PreemptiveStateMachine.__init__(self,outcomes=['endTotem','problem'])
        with self:      
            PreemptiveStateMachine.addPreemptive('EndMatchPreemption',
                                             EndMatchPreempter(-5.0),
                                             transitions={'endMatch':'endTotem'})

            
            pose = TotemPose(0.070, 0.280, -pi/2,table_half)
            PreemptiveStateMachine.add('EnterTotem',
                      AmbiOmniDirectOrder(pose.x, pose.y, pose.h),
                      transitions={'succeeded':'OpenClaws', 'timeout':'Debloque'})
            #as initial state is not the preemptive one, it is necessary to add the information here !
            self.setInitialState('EnterTotem')
            
            PreemptiveStateMachine.add('OpenClaws',
                    TotemClawState(Robot2012.FINGER_CLOSE,Robot2012.FINGER_CLOSE,
                                   Robot2012.CLAW_OPEN, Robot2012.CLAW_TOTEM,
                                   table_half),
                    transitions={'succeeded':'SlashTotem', 'timeout':'SlashTotem'})

            
            pose = TotemPose(0.500,0.125,-pi/2,table_half)
            PreemptiveStateMachine.add('SlashTotem',
                      AmbiOmniDirectOrder_cpoint(0.090,0,0,
                                                 pose.x, pose.y, pose.h),
                      transitions={'succeeded':'SetStratInfo_TotemFinished', 'timeout':'Debloque'})
            
            PreemptiveStateMachine.add('SetStratInfo_TotemFinished',
                      SetStratInfoState('topCloseTotemFull',False),
                      transitions={'ok':'CloseASide'})
            
            PreemptiveStateMachine.add('CloseASide',
                      TotemClawState(Robot2012.FINGER_CLOSE,Robot2012.FINGER_OPEN,
                                          Robot2012.CLAW_CLOSE, Robot2012.CLAW_OPEN,
                                          table_half),
                      transitions={'succeeded':'EndSlashTotem', 'timeout':'EndSlashTotem'})  
            
            pose = TotemPose(0.750,0.125,-pi/2,table_half)
            PreemptiveStateMachine.add('EndSlashTotem',
                      AmbiOmniDirectOrder_cpoint(0.090,0,0,
                                                 pose.x, pose.y, pose.h),
                      transitions={'succeeded':'PushABit', 'timeout':'Debloque'})
            
            pose = TotemPose(0.750,0.000,-pi/2+pi/12,table_half)
            PreemptiveStateMachine.add('PushABit',
                      AmbiOmniDirectOrder(pose.x, pose.y, pose.h),
                      transitions={'succeeded':'endTotem', 'timeout':'Debloque'})
             
            #cas d'erreur
            PreemptiveStateMachine.add('Debloque',
                      DeblocReloc(),
                      transitions={'endDeblocReloc':'problem'})



#you should not use this state, please see user states at the end of file
class ThrowUpCloseTotem(PreemptiveStateMachine):
    def __init__(self, table_half):
        PreemptiveStateMachine.__init__(self,outcomes=['end','problem'])
        with self:      
            PreemptiveStateMachine.addPreemptive('EndMatchPreemption',
                                             EndMatchPreempter(-5.0),
                                             transitions={'endMatch':'end'})
            
                                    
            PreemptiveStateMachine.add('OpenClawMore',
                      TotemClawState(Robot2012.FINGER_CLOSE,Robot2012.FINGER_OPEN,
                                           Robot2012.CLAW_CLOSE, Robot2012.CLAW_OPEN,
                                           table_half),
                      transitions={'succeeded':'ThrowUp', 'timeout':'ThrowUp'})  
            self.setInitialState('OpenClawMore')
            
            pose = TotemPose(1.050,0.150,0,table_half)
            cpoint = TotemPose(0,0.150,0, table_half)
            PreemptiveStateMachine.add('ThrowUp',
                      AmbiOmniDirectOrder_cpoint(cpoint.x, cpoint.y, cpoint.h,
                                                 pose.x, pose.y, pose.h),
                      transitions={'succeeded':'ThrowUpFinish', 'timeout':'Back'})
            
            pose = TotemPose(1.050,-0.150,-pi/6,table_half)
            cpoint = TotemPose(0,-0.150,0, table_half)
            PreemptiveStateMachine.add('ThrowUpFinish',
                      AmbiOmniDirectOrder_cpoint(cpoint.x, cpoint.y, cpoint.h,
                                                 pose.x, pose.y, pose.h),
                      transitions={'succeeded':'SetStratInfo_ThrowUpFinished', 'timeout':'BackFinish'})
            
            PreemptiveStateMachine.add('SetStratInfo_ThrowUpFinished',
                      SetStratInfoState('closeFreeGoldbarInPosition', False),
                      transitions={'ok':'OpenClawABit'})
            
            PreemptiveStateMachine.add('OpenClawABit',
                      TotemClawState(Robot2012.FINGER_HALF_CLOSE,Robot2012.FINGER_OPEN,
                                           Robot2012.CLAW_HALF_CLOSE, Robot2012.CLAW_OPEN,
                                           table_half),
                      transitions={'succeeded':'BackFinish', 'timeout':'ThrowUp'}) 
            
            pose = TotemPose(1.000,-0.150, 0, table_half)
            cpoint = TotemPose(0,-0.150,0, table_half)
            PreemptiveStateMachine.add('BackFinish',
                      AmbiOmniDirectOrder_cpoint(cpoint.x, cpoint.y, cpoint.h,
                                                 pose.x, pose.y, pose.h),
                      transitions={'succeeded':'Back', 'timeout':'Back'})
                                       
            pose = TotemPose(0.950,0.150,-pi/2,table_half)
            PreemptiveStateMachine.add('Back',
                      AmbiOmniDirectOrder(pose.x, pose.y, pose.h),
                      transitions={'succeeded':'CloseFingersAndClaws', 'timeout':'Debloque'})
            
            PreemptiveStateMachine.add('CloseFingersAndClaws',
                      FingerClawState('close'),
                      transitions={'succeeded':'end', 'timeout':'end'}) 

            #cas d'erreur
            PreemptiveStateMachine.add('Debloque',
                      DeblocReloc(),
                      transitions={'endDeblocReloc':'problem'})


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

class ThrowUpBotCloseTotem(ThrowUpCloseTotem):
    def __init__(self):
        ThrowUpCloseTotem.__init__(self,"bot_close")

#######################################################################################################
