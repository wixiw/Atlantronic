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
                                             EndMatchPreempter(-Robot2012.END_GAME_DELAY),
                                             transitions={'endMatch':'endClean'})
                                       
            PreemptiveStateMachine.add('OpenFinger',
                      FingersOnlyState('open'), 
                      transitions={'succeeded':'GotoFirstTotemCoin', 'timeout':'GotoFirstTotemCoin'})
            #as initial state is not the preemptive one, it is necessary to add the information here !
            self.setInitialState('OpenFinger')
            
            #Catia a dit 426mm en y pour le CDG quand on slash les doigts ouverts
            poseFirst = TotemPose(0.080, 0.480,-pi/4,table_half)
            PreemptiveStateMachine.add('GotoFirstTotemCoin',
                      AmbiOmniDirectOrder(poseFirst.x, poseFirst.y, poseFirst.h, vmax=0.3),
                      transitions={'succeeded':'TotemClean', 'timeout':'Debloque'})
            
            poseClean = TotemPose(0.600,0.450,-pi/4,table_half)
            PreemptiveStateMachine.add('TotemClean',
                      AmbiOmniDirectOrder(poseClean.x, poseClean.y, poseClean.h),
                      transitions={'succeeded':'CloseFingersABit', 'timeout':'Retry'})
            
            PreemptiveStateMachine.add('CloseFingersABit',
                      TotemClawState(0.25, Robot2012.FINGER_OPEN,
                                     Robot2012.CLAW_CLOSE, Robot2012.CLAW_CLOSE, table_half), 
                      transitions={'succeeded':'PushALot', 'timeout':'PushALot'})
            
            PreemptiveStateMachine.add('PushALot',
                      AmbiOmniDirectOrder(1.10,0.150,pi/12),
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
                      Rewind(1.0),
                      transitions={'succeeded':'RetryFirstTotemCoin', 'timeout':'Debloque'})
            PreemptiveStateMachine.add('RetryFirstTotemCoin',
                      AmbiOmniDirectOrder(poseFirst.x, poseFirst.y + 0.030, poseFirst.h, 0.2),
                      transitions={'succeeded':'RetryTotemClean', 'timeout':'Debloque'})
            PreemptiveStateMachine.add('RetryTotemClean',
                      AmbiOmniDirectOrder(poseClean.x, poseClean.y + 0.030, poseClean.h, 0.2),
                      transitions={'succeeded':'PushALot', 'timeout':'Debloque'})
            
                        
            #cas d'erreur
            PreemptiveStateMachine.add('Debloque',
                      DeblocReloc(),
                      transitions={'endDeblocReloc':'problem'}) 
            

#you should not use this state, please see user states at the end of file
# TODO : a mettre a jour avec le retry
# TODO : a tester avant usage
class WorkCloseTotem(PreemptiveStateMachine):
    def __init__(self, table_half):
        PreemptiveStateMachine.__init__(self,outcomes=['endTotem','problem'])
        with self:      
            PreemptiveStateMachine.addPreemptive('EndMatchPreemption',
                                             EndMatchPreempter(-Robot2012.END_GAME_DELAY),
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
            
            #y=0.204 est la distance theorique entre le centre du totem et le Robot dans Catia 
            # lorsque la griffe est sortie a 90 deg et que le robot est parrallele au totem
            pose = TotemPose(0.200, 
                             0.204 - Robot2012.CDG_POSE.x + Robot2012.TOTEM_CLAW_MARGIN , 
                             -pi/2,
                             table_half)
            PreemptiveStateMachine.add('EnterTotem',
                      AmbiOmniDirectOrder(pose.x, pose.y, pose.h),
                      transitions={'succeeded':'EndSlashTotem', 'timeout':'Debloque'})
              
            
            pose = TotemPose(0.750,
                             0.204 - Robot2012.CDG_POSE.x + Robot2012.TOTEM_CLAW_MARGIN , 
                             -pi/2,
                             table_half)
            PreemptiveStateMachine.add('EndSlashTotem',
                      AmbiOmniDirectOrder(pose.x, pose.y, pose.h),
                      transitions={'succeeded':'SetStratInfo_TotemFinished', 'timeout':'Debloque'})
            
            PreemptiveStateMachine.add('SetStratInfo_TotemFinished',
                      SetStratInfoState('topCloseTotemFull',False),
                      transitions={'ok':'PushABit'})
            
            pose = TotemPose(0.750,0.120,-pi/2,table_half)
            PreemptiveStateMachine.add('PushABit',
                      AmbiOmniDirectOrder(pose.x, pose.y, pose.h),
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
                                             EndMatchPreempter(-Robot2012.END_GAME_DELAY),
                                             transitions={'endMatch':'endTotem'})
            
            #en cas de modification, modifier le retry aussi
            poseEnter = TotemPose(0.070, 
                             0.204 - Robot2012.CDG_POSE.x + Robot2012.TOTEM_CLAW_MARGIN,
                             -pi/2,
                             table_half)
            PreemptiveStateMachine.add('EnterTotem',
                      AmbiOmniDirectOrder(poseEnter.x, poseEnter.y, poseEnter.h),
                      transitions={'succeeded':'OpenClaws', 'timeout':'Debloque'})
            #as initial state is not the preemptive one, it is necessary to add the information here !
            self.setInitialState('EnterTotem')
            
            PreemptiveStateMachine.add('OpenClaws',
                    TotemClawState(Robot2012.FINGER_CLOSE,Robot2012.FINGER_CLOSE,
                                   Robot2012.CLAW_OPEN, Robot2012.CLAW_TOTEM,
                                   table_half),
                    transitions={'succeeded':'SlashTotem', 'timeout':'SlashTotem'})

            #en cas de modification, modifier le retry aussi
            poseTotem = TotemPose(0.500,
                             0.204 - Robot2012.CDG_POSE.x + Robot2012.TOTEM_CLAW_MARGIN,
                             -pi/2,
                             table_half)
            PreemptiveStateMachine.add('SlashTotem',
                      AmbiOmniDirectOrder(poseTotem.x, poseTotem.y, poseTotem.h),
                      transitions={'succeeded':'SetStratInfo_TotemFinished', 'timeout':'Retry'})
            
            PreemptiveStateMachine.add('SetStratInfo_TotemFinished',
                      SetStratInfoState('topCloseTotemFull',False),
                      transitions={'ok':'CloseASide'})
            
            PreemptiveStateMachine.add('CloseASide',
                      TotemClawState(Robot2012.FINGER_CLOSE,Robot2012.FINGER_OPEN,
                                          Robot2012.CLAW_CLOSE, Robot2012.CLAW_OPEN,
                                          table_half),
                      transitions={'succeeded':'EndSlashTotem', 'timeout':'EndSlashTotem'})  
            
            pose = TotemPose(0.750,
                             0.204 - Robot2012.CDG_POSE.x + Robot2012.TOTEM_CLAW_MARGIN,
                             -pi/2,
                             table_half)
            PreemptiveStateMachine.add('EndSlashTotem',
                      AmbiOmniDirectOrder(pose.x, pose.y, pose.h),
                      transitions={'succeeded':'PushABit', 'timeout':'GoBack'})
            
            #si ca bloque va a destination en s'ecartant
            PreemptiveStateMachine.add('GoBack',
                      BackwardOrder(0.070),
                      transitions={'succeeded':'GoAway', 'timeout':'GoAway'})
            pose = TotemPose(0.750,
                             0.204 - Robot2012.CDG_POSE.x + Robot2012.TOTEM_CLAW_MARGIN + 0.050,
                             -pi/2,
                             table_half)
            PreemptiveStateMachine.add('GoAway',
                      AmbiOmniDirectOrder(pose.x, pose.y, pose.h),
                      transitions={'succeeded':'PushABit', 'timeout':'Debloque'})
            
            
            pose = TotemPose(0.730,-0.050,-pi/2+pi/12,table_half)
            PreemptiveStateMachine.add('PushABit',
                      AmbiOmniDirectOrder(pose.x, pose.y, pose.h),
                      transitions={'succeeded':'WaitPump', 'timeout':'WaitPump'})
            
            PreemptiveStateMachine.add('WaitPump',
                      WaiterState(0.3),
                      transitions={'timeout':'ClosePump'})  
            
            PreemptiveStateMachine.add('ClosePump',
                      TotemClawState(Robot2012.FINGER_CLOSE,Robot2012.FINGER_HALF_CLOSE,
                                          Robot2012.CLAW_CLOSE, Robot2012.CLAW_HALF_CLOSE,
                                          table_half),
                      transitions={'succeeded':'OpenPump', 'timeout':'OpenPump'})  
            
            PreemptiveStateMachine.add('OpenPump',
                      TotemClawState(Robot2012.FINGER_CLOSE,Robot2012.FINGER_HALF_CLOSE,
                                          Robot2012.CLAW_CLOSE, Robot2012.CLAW_HALF_CLOSE,
                                          table_half),
                      transitions={'succeeded':'endTotem', 'timeout':'endTotem'})  
            
             
            #si on a rate le slash, on s'est peut etre pris dans le totem
            #on recommence une fois en partant de plus loin   
            PreemptiveStateMachine.add('Retry',
                      BackwardOrder(0.060),
                      transitions={'succeeded':'RetryEnterTotem', 'timeout':'Debloque'})
            
            poseEnter = TotemPose(0.070, 
                             0.204 - Robot2012.CDG_POSE.x + Robot2012.TOTEM_CLAW_MARGIN + 0.060,
                             -pi/2,
                             table_half)
            PreemptiveStateMachine.add('RetryEnterTotem',
                      AmbiOmniDirectOrder(poseEnter.x, poseEnter.y + 0.030, poseEnter.h, vmax=0.2),
                      transitions={'succeeded':'RetryTotemSlash', 'timeout':'Debloque'})
            
            poseTotem = TotemPose(0.500,
                 0.204 - Robot2012.CDG_POSE.x + Robot2012.TOTEM_CLAW_MARGIN + 0.040,
                 -pi/2,
                 table_half)
            PreemptiveStateMachine.add('RetryTotemSlash',
                      AmbiOmniDirectOrder(poseTotem.x, poseTotem.y , poseTotem.h, vmax=0.2),
                      transitions={'succeeded':'SetStratInfo_TotemFinished', 'timeout':'Debloque'}) 
             
             
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
                                             EndMatchPreempter(-Robot2012.END_GAME_DELAY),
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
                      transitions={'succeeded':'SetStratInfo_ThrowUpFinished', 'timeout':'OpenClawABit'})
            
            PreemptiveStateMachine.add('SetStratInfo_ThrowUpFinished',
                      SetStratInfoState('closeFreeGoldbarInPosition', False),
                      transitions={'ok':'OpenClawABit'})
            
            PreemptiveStateMachine.add('OpenClawABit',
                      TotemClawState(Robot2012.FINGER_HALF_CLOSE,Robot2012.FINGER_OPEN,
                                           Robot2012.CLAW_HALF_CLOSE, Robot2012.CLAW_OPEN,
                                           table_half),
                      transitions={'succeeded':'BackFinish', 'timeout':'BackFinish'}) 
            
            pose = TotemPose(1.000,-0.150, 0, table_half)
            cpoint = TotemPose(0,-0.150,0, table_half)
            PreemptiveStateMachine.add('BackFinish',
                      AmbiOmniDirectOrder_cpoint(cpoint.x, cpoint.y, cpoint.h,
                                                 pose.x, pose.y, pose.h),
                      transitions={'succeeded':'Back', 'timeout':'Back'})
                                       
            pose = TotemPose(0.900,0.150,-pi/2,table_half)
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





class AntiWorkFullBotTotem(PreemptiveStateMachine):
    def __init__(self):
        PreemptiveStateMachine.__init__(self,outcomes=['endTotem','problem'])
        with self:      
            PreemptiveStateMachine.addPreemptive('EndMatchPreemption',
                                             EndMatchPreempter(-Robot2012.END_GAME_DELAY),
                                             transitions={'endMatch':'endTotem'})
            
            #en cas de modification, modifier le retry aussi
            PreemptiveStateMachine.add('PrepareTotem',
                      AmbiOmniDirectOrder(0.850, -0.204 + Robot2012.CDG_POSE.x - 5*Robot2012.TOTEM_CLAW_MARGIN, pi/2+pi/5),
                      transitions={'succeeded':'SlashEnter', 'timeout':'Debloque'})
            #as initial state is not the preemptive one, it is necessary to add the information here !
            self.setInitialState('PrepareTotem')
            
            PreemptiveStateMachine.add('SlashEnter',
                      AmbiOmniDirectOrder(-0.750, -0.204 + Robot2012.CDG_POSE.x - 5*Robot2012.TOTEM_CLAW_MARGIN,  pi/2+pi/5, 0.6),
                      transitions={'succeeded':'OpenClaws', 'timeout':'Debloque'})
            
            PreemptiveStateMachine.add('OpenClaws',
                    AmbiClawFingerOrder(Robot2012.FINGER_CLOSE,Robot2012.FINGER_CLOSE,
                                   Robot2012.CLAW_CLOSE, Robot2012.CLAW_OPEN),
                    transitions={'succeeded':'EnterTotem', 'timeout':'EnterTotem'})
            
            PreemptiveStateMachine.add('EnterTotem',
                      AmbiOmniDirectOrder(-0.500, -0.204 + Robot2012.CDG_POSE.x - Robot2012.TOTEM_CLAW_MARGIN,  pi/2, 0.6),
                      transitions={'succeeded':'SlashTotem1', 'timeout':'SlashTotem1'})
            
            PreemptiveStateMachine.add('SlashTotem1',
                      AmbiOmniDirectOrder(-0.200,-0.204 + Robot2012.CDG_POSE.x - Robot2012.TOTEM_CLAW_MARGIN, pi/2, 0.4),
                      transitions={'succeeded':'CloseClaws', 'timeout':'Debloque'})
            
            PreemptiveStateMachine.add('CloseClaws',
                    AmbiClawFingerOrder(Robot2012.FINGER_CLOSE,Robot2012.FINGER_CLOSE,
                                   Robot2012.CLAW_CLOSE, Robot2012.CLAW_CLOSE),
                    transitions={'succeeded':'AntiWorkCloseTotem', 'timeout':'AntiWorkCloseTotem'})
            
            PreemptiveStateMachine.add('AntiWorkCloseTotem',
                      AntiWorkBotCloseTotem(),
                      transitions={'endTotem':'ThrowUpBotCloseTotem', 'problem':'problem'})
            
            PreemptiveStateMachine.add('ThrowUpBotCloseTotem',
                      ThrowUpBotCloseTotem(),
                      transitions={'end':'endTotem', 'problem':'problem'})


            #cas d'erreur
            PreemptiveStateMachine.add('Debloque',
                      DeblocReloc(),
                      transitions={'endDeblocReloc':'endTotem'})




class AntiWorkFullTopTotem(PreemptiveStateMachine):
    def __init__(self):
        PreemptiveStateMachine.__init__(self,outcomes=['endTotem','problem'])
        with self:      
            PreemptiveStateMachine.addPreemptive('EndMatchPreemption',
                                             EndMatchPreempter(-Robot2012.END_GAME_DELAY),
                                             transitions={'endMatch':'endTotem'})
            
            #en cas de modification, modifier le retry aussi
            PreemptiveStateMachine.add('PrepareTotem',
                      AmbiOmniDirectOrder(0.850, 0.204 - Robot2012.CDG_POSE.x + 5*Robot2012.TOTEM_CLAW_MARGIN, -pi/2-pi/5),
                      transitions={'succeeded':'SlashEnter', 'timeout':'Debloque'})
            #as initial state is not the preemptive one, it is necessary to add the information here !
            self.setInitialState('PrepareTotem')
            
            PreemptiveStateMachine.add('SlashEnter',
                      AmbiOmniDirectOrder(-0.750, +0.204 - Robot2012.CDG_POSE.x + 5*Robot2012.TOTEM_CLAW_MARGIN,  -pi/2-pi/5, 0.6),
                      transitions={'succeeded':'OpenClaws', 'timeout':'Debloque'})
            
            PreemptiveStateMachine.add('OpenClaws',
                    AmbiClawFingerOrder(Robot2012.FINGER_CLOSE,Robot2012.FINGER_CLOSE,
                                   Robot2012.CLAW_OPEN, Robot2012.CLAW_CLOSE),
                    transitions={'succeeded':'EnterTotem', 'timeout':'EnterTotem'})
            
            PreemptiveStateMachine.add('EnterTotem',
                      AmbiOmniDirectOrder(-0.500, +0.204 - Robot2012.CDG_POSE.x + Robot2012.TOTEM_CLAW_MARGIN,  -pi/2, 0.6),
                      transitions={'succeeded':'SlashTotem1', 'timeout':'SlashTotem1'})
            
            PreemptiveStateMachine.add('SlashTotem1',
                      AmbiOmniDirectOrder(0.-200,0.204 - Robot2012.CDG_POSE.x + Robot2012.TOTEM_CLAW_MARGIN, -pi/2, 0.4),
                      transitions={'succeeded':'CloseClaws', 'timeout':'Debloque'})
            
            PreemptiveStateMachine.add('CloseClaws',
                    AmbiClawFingerOrder(Robot2012.FINGER_CLOSE,Robot2012.FINGER_CLOSE,
                                   Robot2012.CLAW_CLOSE, Robot2012.CLAW_CLOSE),
                    transitions={'succeeded':'AntiWorkCloseTotem', 'timeout':'AntiWorkCloseTotem'})
            
            PreemptiveStateMachine.add('AntiWorkCloseTotem',
                      AntiWorkTopCloseTotem(),
                      transitions={'endTotem':'ThrowUpTopCloseTotem', 'problem':'problem'})
            
            PreemptiveStateMachine.add('ThrowUpTopCloseTotem',
                      ThrowUpTopCloseTotem(),
                      transitions={'end':'endTotem', 'problem':'problem'})


            #cas d'erreur
            PreemptiveStateMachine.add('Debloque',
                      DeblocReloc(),
                      transitions={'endDeblocReloc':'endTotem'})




