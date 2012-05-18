#!/usr/bin/env python

#libraries for ROS
import roslib; roslib.load_manifest('arp_master')
from arp_master import *

class MiddleGame(PreemptiveStateMachine):
    def __init__(self):
        PreemptiveStateMachine.__init__(self,outcomes=['endMiddleGame'])
        with self:      
            PreemptiveStateMachine.addPreemptive('EndMatchPreemption',
                                             EndMatchPreempter(-Robot2012.END_GAME_DELAY),
                                             transitions={'endMatch':'endMiddleGame'})
    
   ##
   # Top TOTEM         
            PreemptiveStateMachine.add('CleanTopCloseTotem',
                      CleanTopCloseTotem(),
                      transitions={'endClean':'TopCloseTotem', 'problem':'CloseBottleAndCoin'})
            #as initial state is not the preemptive one, it is necessary to add the information here !
            self.setInitialState('CleanTopCloseTotem')
            
            PreemptiveStateMachine.add('TopCloseTotem',
                      AntiWorkTopCloseTotem(),
                      transitions={'endTotem':'ThrowUpTopCloseTotem', 'problem':'CloseBottleAndCoin'})

            PreemptiveStateMachine.add('ThrowUpTopCloseTotem',
                      ThrowUpTopCloseTotem(),
                      transitions={'end':'CloseBottleAndCoin', 'problem':'CloseBottleAndCoin'})
    # Top TOTEM        
    ##
            
            PreemptiveStateMachine.add('CloseBottleAndCoin',
                      CloseBottleState(),
                      transitions={'endBottle':'CleanBotCloseTotem', 'problem':'CleanBotCloseTotem'})
         
#            PreemptiveStateMachine.add('PrepareBotTotem',
#                      AmbiOmniDirectOrder(0.550, -0.650, pi/2),
#                      transitions={'succeeded':'CleanBotCloseTotem', 'timeout':'Debloque'})  
            ####### A MODIFIER (OU PAS) POUR ASSURER NON ARRET 
            
    ##
    # Bot TOTEM         

            PreemptiveStateMachine.add('CleanBotCloseTotem',
                      CleanBotCloseTotem(),
                      transitions={'endClean':'BotCloseTotem', 'problem':'MiddleObject'})
                       
            PreemptiveStateMachine.add('BotCloseTotem',
                      AntiWorkBotCloseTotem(),
                      transitions={'endTotem':'ThrowUpBotCloseTotem', 'problem':'MiddleObject'})
            
            PreemptiveStateMachine.add('ThrowUpBotCloseTotem',
                      ThrowUpBotCloseTotem(),
                      transitions={'end':'MiddleObject', 'problem':'MiddleObject'})        
    # Bot TOTEM        
    ##
        
        
#            PreemptiveStateMachine.add('Turn',
#                      AmbiTurnOrder(0.0),
#                      transitions={'succeeded':'CloseBottleAndCoin', 'timeout':'CloseBottleAndCoin'})
#        
#
#            
            
            PreemptiveStateMachine.add('MiddleObject',
                      MiddleObjects(),
                      transitions={'end':'FarBottle', 'problem':'endMiddleGame'})
             ####### A MODIFIER POUR ASSURER NON ARRET
            
            PreemptiveStateMachine.add('FarBottle',
                      FarBottleState(),
                      transitions={'endBottle':'BackFromMiddleObjects', 'problem':'endMiddleGame'})
             ####### A MODIFIER POUR ASSURER NON ARRET
            
            PreemptiveStateMachine.add('BackFromMiddleObjects',
                      BackFromMiddleObjects(),
                      transitions={'end':'ThrowUp', 'problem':'endMiddleGame'})
             ####### A MODIFIER POUR ASSURER NON ARRET
            
            PreemptiveStateMachine.add('ThrowUp',
                      AmbiOmniDirectOrder(1.100,0.050,pi/5),
                      transitions={'succeeded':'SetStratInfo_ThrowUpFinished', 'timeout':'Back'})

            PreemptiveStateMachine.add('SetStratInfo_ThrowUpFinished',
                      SetStratInfoState('closeFreeGoldbarInPosition', False),
                      transitions={'ok':'Back'})
    
            PreemptiveStateMachine.add('Back',
                      AmbiOmniDirectOrder(0.950,-0.100,pi/2),
                      transitions={'succeeded':'CloseFingersAndClaws', 'timeout':'CloseFingersAndClaws'})
            ####### A MODIFIER POUR ASSURER NON ARRET
            
            PreemptiveStateMachine.add('CloseFingersAndClaws',
                      FingerClawState('close'),
                      transitions={'succeeded':'endMiddleGame', 'timeout':'endMiddleGame'}) 
            ####### A MODIFIER POUR ASSURER NON ARRET


            
            #cas d'erreur
            PreemptiveStateMachine.add('Debloque',
                      DeblocReloc(),
                      transitions={'endDeblocReloc':'endMiddleGame'})
            ####### A MODIFIER POUR ASSURER NON ARRET
