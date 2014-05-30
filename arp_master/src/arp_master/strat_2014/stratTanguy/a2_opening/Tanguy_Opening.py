#!/usr/bin/env python
# -*- coding: latin-1 -*-

#libraries for ROS
import roslib; roslib.load_manifest('arp_master')
from arp_master import *
from arp_master.strat_2014 import *

class Opening(PreemptiveStateMachine):
    def __init__(self):
        PreemptiveStateMachine.__init__(self, outcomes=['endOpening', 'motionBlocked', 'askSelector', 'nearlyEndMatch'])
        with self:
            PreemptiveStateMachine.addPreemptive('EndMatchPreemption',
                                             EndMatchPreempter(-Robot2014.SWITCH_TO_EOG_DELAY),
                                             transitions={'endMatch':'nearlyEndMatch'})
            
            PreemptiveStateMachine.add('EscapeStartArea',
                      AmbiOmniDirectOrder2Pass(Table2014.P_YOU_HOU, vmax=Robot2014.motionSpeeds['Fast'], vpasse=-1),
                      transitions={'succeeded':'GoToSFT', 'timeout':'RetryEscapeStartArea'})
            self.setInitialState('EscapeStartArea')
            
            PreemptiveStateMachine.add('RetryEscapeStartArea',
                      AmbiOmniDirectOrder2Pass(Pose2D(Table2014.P_YOU_HOU.x, Table2014.P_YOU_HOU.y -0.050, Table2014.P_YOU_HOU.theta), vmax=Robot2014.motionSpeeds['Carefull'] ,vpasse=-1),
                      transitions={'succeeded':'GoToSFT', 'timeout':'EscapeStartArea'})
            
# Go to Self Fire Top
            PreemptiveStateMachine.add('GoToSFT',
                      AmbiOmniDirectOrder2Pass(Pose2D(0.650 + Robot2014.FRONT_SIDE.x, 0.300, -5*pi/6), vmax=Robot2014.motionSpeeds['Fast'],vpasse=-1),
                      transitions={'succeeded':'GoMid', 'timeout':'EscapeStartArea'}) #si on y arrive pas on repart au depart et on recommence
            
            # -pi/2 -0.05 car on veut eviter que ca tourne tout seul du mauvais cote
            PreemptiveStateMachine.add('GoMid',
                      AmbiOmniDirectOrder2Pass(Pose2D(0.0, 0.400, -pi/2 + 0.05), vmax=Robot2014.motionSpeeds['Average'],vpasse=-1),
                      transitions={'succeeded':'GoToOpponentTopFire', 'timeout':'PrepareFrescos'}) #si on y arrive pas on repart au depart et on recommence


            ## Push Opponent Top Fire, SLOWLY (parce qu'on est chez lui quand même ...)
            PreemptiveStateMachine.add('GoToOpponentTopFire',
                      AmbiOmniDirectOrder2(Pose2D(-0.600, 0.400, -pi),vmax=Robot2014.motionSpeeds['Carefull']),
                      transitions={'succeeded':'PrepareFrescos', 'timeout':'DeblocProvocatedCollision'}) 
            
            PreemptiveStateMachine.add('DeblocProvocatedCollision',
                      OpenLoopOrder(0.3,0.0,0.0, duration=0.4),
                      transitions={'succeeded':'PrepareFrescos', 'timeout':'PrepareFrescos'})
            
            
            #si on y arrive pas, tant pis on passe l'action on va coller
            
# Shoot Opponent Mammoth
#            PreemptiveStateMachine.add('TargetShoot',
#                      AmbiShootMammoth('Left', p_opponent_side = True),
#                      transitions={'succeeded':'PrepareFrescos', 'failed':'nearlyEndMatch', 'almostEndGame':'nearlyEndMatch'})

# Go to Frescos entry point
            PreemptiveStateMachine.add('PrepareFrescos',
                      AmbiOmniDirectOrder2(StickFrescosState.getEntryYellowPoseStatic(), vmax=Robot2014.motionSpeeds['Fast']),
                      transitions={'succeeded':'StickFrescos', 'timeout':'RecoverPrepareStickFrescos'})   #si on y arrive pas, on va au self shoot point
            
            PreemptiveStateMachine.add('RecoverPrepareStickFrescos',
                      AmbiOmniDirectOrder2(Table2014.P_YOU_HOU, vmax=Robot2014.motionSpeeds['Carefull']),
                      transitions={'succeeded':'GoToSFM', 'timeout':'GoToSFT'})
            
#Stick Frescos
            PreemptiveStateMachine.add('StickFrescos',
                      StickFrescosState(),
                      transitions={'succeeded':'PrepareRecalYAfterRush', 'failed':'EmmergencyEscapeFrescos', 'almostEndGame':'nearlyEndMatch' }) # si on y arrive pas on part au point d'urgence de sortie
            
            PreemptiveStateMachine.add('EmmergencyEscapeFrescos',
                      AmbiOmniDirectOrder2(Pose2D(0 , 0.400, -5/6*pi), vmax=Robot2014.motionSpeeds['Carefull']),
                      transitions={'succeeded':'PrepareRecalYAfterRush', 'timeout':'PrepareRecalYAfterRush' })

#Recalage apres le rush
            PreemptiveStateMachine.add('PrepareRecalYAfterRush',
                      AmbiOmniDirectOrder2(Recal2014_Y_Fruitbasket.getEntryYellowPoseStatic(), vmax=Robot2014.motionSpeeds['Fast']),
                      transitions={'succeeded':'RecalYAfterRush', 'timeout':'GoToSFM'})

            PreemptiveStateMachine.add('RecalYAfterRush',
                      Recal2014_Y_Fruitbasket(),
                      transitions={'succeeded':'GotoSelfShootPoint',
                                   'failed':'GotoSelfShootPoint', 
                                   'almostEndGame':'nearlyEndMatch'})  

# Go to Self Shoot Point
            PreemptiveStateMachine.add('GotoSelfShootPoint',
                      AmbiOmniDirectOrder2(AmbiShootMammoth.getEntryYellowPoseStatic('Left', p_opponent_side = False)),
                      transitions={'succeeded':'GoToSFM', 'timeout':'GoToSFM'}) # si on y arrive pas on passe a la suite

# Shoot Self Mammoth
#            PreemptiveStateMachine.add('TargetShoot',
#                      AmbiShootMammoth('Right', p_opponent_side = False),
#                      transitions={'succeeded':'GoToSFM', 'failed':'nearlyEndMatch', 'almostEndGame':'nearlyEndMatch'})


# Go to Self Fire Mid
            PreemptiveStateMachine.add('GoToSFM',
                      AmbiOmniDirectOrder2Pass(Pose2D(1.200, 0.000, -pi/2), vmax=Robot2014.motionSpeeds['Fast'], vpasse=0.3),
                      transitions={'succeeded':'WaypointBeforeSmurf', 'timeout':'EmmergencyPointSFM'}) #Si on y arrive pas, on va au dessus de la torche mobile
            
            
            PreemptiveStateMachine.add('EmmergencyPointSFM',
                      AmbiOmniDirectOrder2Pass(Pose2D(0.600, 0.200, -pi/2), vmax=Robot2014.motionSpeeds['Carefull'], vpasse=0.3),
                      transitions={'succeeded':'WaypointBeforeSmurf', 'timeout':'GoToSFB'}) #Si on y arrive pas, on va au Self fire bot

            #faut taper fort dans les fire d'ou le 1.0
            PreemptiveStateMachine.add('WaypointBeforeSmurf',
                      AmbiOmniDirectOrder2Pass(Pose2D(1.100, -0.200, -pi/2), vmax=Robot2014.motionSpeeds['Fast'], vpasse=0.3),
                      transitions={'succeeded':'GoToSmurfPoint', 'timeout':'GoToSmurfPoint'}) #Si on y arrive pas, on va au dessus de la torche mobile                       

# Go to Smurf Point to go to Self Fire Bot
            PreemptiveStateMachine.add('GoToSmurfPoint',
                      AmbiOmniDirectOrder2Pass(Pose2D(0.400, -0.300, -pi/2), vmax=Robot2014.motionSpeeds['Fast'], vpasse=0.3),
                      transitions={'succeeded':'GoToSFB', 'timeout':'PrepareRecalY'}) #Si on y arrive pas, on va au dessus de la torche mobile

# Go to Self Fire Bot
            PreemptiveStateMachine.add('GoToSFB',
                      AmbiOmniDirectOrder2Pass(Pose2D(0.600, -0.600, 0), vmax=Robot2014.motionSpeeds['Fast'], vpasse=0.3),
                      transitions={'succeeded':'PrepareRecalY', 'timeout':'EmergencyMiddlePoint'}) #Si on y arrive pas, on va au Emergency Middle Point

            PreemptiveStateMachine.add('EmergencyMiddlePoint',
                      AmbiOmniDirectOrder2(Pose2D(1.200, 0.200, -pi/2), vmax=Robot2014.motionSpeeds['Carefull']),
                      transitions={'succeeded':'PrepareRecalY', 'timeout':'GoToOpponentTorchBot'}) #Si on y arrive pas, on retourne au opponent shoot point
            
#Ordres de recalage Top

            PreemptiveStateMachine.add('PrepareRecalY',
                      AmbiOmniDirectOrder2(Recal2014_Y_Fruitbasket.getEntryYellowPoseStatic(), vmax=Robot2014.motionSpeeds['Fast']),
                      transitions={'succeeded':'RecalY', 'timeout':'GoToSelfTorchMid'})

            PreemptiveStateMachine.add('RecalY',
                      Recal2014_Y_Fruitbasket(),
                      transitions={'succeeded':'GoToSelfTorchMid',
                                   'failed':'GoToSelfTorchMid', 
                                   'almostEndGame':'nearlyEndMatch'})  
             
             
             
            PreemptiveStateMachine.add('GoToSelfTorchMid',
                      AmbiOmniDirectOrder2(Recal2014_X_SelfTorchMid.getEntryYellowPoseStatic(), vmax=Robot2014.motionSpeeds['Average']),
                      transitions={'succeeded':'RecalXSelfTorchMid', 'timeout':'GoToSelfTorchMid'})
            #TODO !!!!!! cas de retry a faire

            PreemptiveStateMachine.add('RecalXSelfTorchMid',
                      Recal2014_X_SelfTorchMid(),
                      transitions={'succeeded':'WaipointFireBot',
                                   'failed':'WaipointFireBot', 
                                   'almostEndGame':'nearlyEndMatch'})  
             

# Go to WaipointFire Bot
            PreemptiveStateMachine.add('WaipointFireBot',
                      AmbiOmniDirectOrder2(Pose2D(0.600, -0.600, -pi/4), vmax=Robot2014.motionSpeeds['Fast']),
                      transitions={'succeeded':'GoToOpponentTorchBot', 'timeout':'GoToSFB'})
            
# Go to Opponent Torch Bot
            PreemptiveStateMachine.add('GoToOpponentTorchBot',
                      AmbiOmniDirectOrder2(Pose2D(-0.200, -0.885, -pi/2), vmax=Robot2014.motionSpeeds['Average']),
                      transitions={'succeeded':'RecalYOpponentTorchBot', 'timeout':'GoToSFB'})

# Push Opponent Torch Bot
            PreemptiveStateMachine.add('RecalYOpponentTorchBot',
                      AmbiRecalOnBorderYellow("DOWN"),
                      transitions={'recaled':'GoToOpponentMobileTorch', 'non-recaled':'GoToSFB','problem':'GoToSFB'})  

# Go to Opponent Mobile Torch
            PreemptiveStateMachine.add('GoToOpponentMobileTorch',
                      AmbiOmniDirectOrder2Pass(Pose2D(-0.500, -0.100, -pi/2), vmax=Robot2014.motionSpeeds['Average'], vpasse=0.3),
                      transitions={'succeeded':'GoToOpponentFireMid', 'timeout':'GoToOpponentEscapePoint'})
            
# Go to Opponent Escape Point
            PreemptiveStateMachine.add('GoToOpponentEscapePoint',
                      AmbiOmniDirectOrder2Pass(Pose2D(0, -0.500, pi/2), vmax=Robot2014.motionSpeeds['Average'], vpasse=0.3),
                      transitions={'succeeded':'GoToSFB', 'timeout':'GoToSFB'})
            
# Go to Opponent Fire Mid
            PreemptiveStateMachine.add('GoToOpponentFireMid',
                      AmbiOmniDirectOrder2Pass(Pose2D(-1.100, -0.200, pi/2), vmax=Robot2014.motionSpeeds['Average'], vpasse=0.3),
                      transitions={'succeeded':'PushOpponentFireMid', 'timeout':'GoToSFB'})
            
# Push Opponent Fire Mid
            PreemptiveStateMachine.add('PushOpponentFireMid',
                      AmbiOmniDirectOrder2Pass(Pose2D(-1.100, 0.000, pi/2), vmax=Robot2014.motionSpeeds['Carefull'], vpasse=0.3),
                      transitions={'succeeded':'GoToOpponentFireBot', 'timeout':'GoToSFB'})
            
# Go to Opponent Fire Bot
            PreemptiveStateMachine.add('GoToOpponentFireBot',
                  AmbiOmniDirectOrder2Pass(Pose2D(-0.700, -0.600, 0), vmax=Robot2014.motionSpeeds['Average'], vpasse=0.3),
                  transitions={'succeeded':'PushOpponentFireBot', 'timeout':'GoToSFB'})
        
# Push Opponent Fire Bot
            PreemptiveStateMachine.add('PushOpponentFireBot',
                  AmbiOmniDirectOrder2Pass(Pose2D(-0.500, -0.600, 0), vmax=Robot2014.motionSpeeds['Average'], vpasse=0.3),
                  transitions={'succeeded':'GoToSFB', 'timeout':'GoToSFB'})       
    
####################################################################################################################
## End of Rush
####################################################################################################################
