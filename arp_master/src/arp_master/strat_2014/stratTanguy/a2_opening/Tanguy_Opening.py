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
                      AmbiOmniDirectOrder2Pass(Table2014.P_IN_FRONT_START_AREA, vmax=Robot2014.motionSpeeds['Fast'], vpasse=-1),
                      transitions={'succeeded':'GoToSFT', 'timeout':'RetryEscapeStartArea'})
            self.setInitialState('EscapeStartArea')
            
            PreemptiveStateMachine.add('RetryEscapeStartArea',
                      AmbiOmniDirectOrder2Pass(Pose2D(Table2014.P_IN_FRONT_START_AREA.x, Table2014.P_IN_FRONT_START_AREA.y -0.050, Table2014.P_IN_FRONT_START_AREA.theta), vmax=Robot2014.motionSpeeds['Carefull'] ,vpasse=-1),
                      transitions={'succeeded':'GoToSFT', 'timeout':'EscapeStartArea'})
            
# Go to Self Fire Top
            PreemptiveStateMachine.add('GoToSFT',
                      AmbiOmniDirectOrder2Pass(Pose2D(0.650 + Robot2014.FRONT_SIDE.x, 0.400, 0), vmax=Robot2014.motionSpeeds['Fast'],vpasse=-1),
                      transitions={'succeeded':'GoMid', 'timeout':'EscapeStartArea'}) #si on y arrive pas on repart au depart et on recommence
            
            # Conserver angle 0 pour reculer tout droit
            PreemptiveStateMachine.add('GoMid',
                      AmbiOmniDirectOrder2Pass(Pose2D(0.0, 0.400, 0), vmax=Robot2014.motionSpeeds['Fast'],vpasse=-1),
                      transitions={'succeeded':'GoToOpponentTopFire', 'timeout':'PrepareFrescos'}) #si on y arrive pas on repart au depart et on recommence


            ## Push Opponent Top Fire, SLOWLY (parce qu'on est chez lui quand même ...)
            PreemptiveStateMachine.add('GoToOpponentTopFire',
                      AmbiOmniDirectOrder2(Pose2D(-0.600, 0.400, 0),vmax=Robot2014.motionSpeeds['Carefull']),
                      transitions={'succeeded':'GotoPrepareOppShoot', 'timeout':'DeblocProvocatedCollision'}) 
            
            PreemptiveStateMachine.add('DeblocProvocatedCollision',
                      OpenLoopOrder(0.3,0.0,0.0, duration=0.4),
                      transitions={'succeeded':'GotoPrepareOppShoot', 'timeout':'GotoPrepareOppShoot'})
            
            
            #si on y arrive pas, tant pis on passe l'action on va coller
            
# Shoot Opponent Mammoth
            PreemptiveStateMachine.add('GotoPrepareOppShoot',
                      AmbiOmniDirectOrder2(AmbiShootMammoth.getEntryYellowPoseStatic('Left', p_opponent_side = True), vmax=Robot2014.motionSpeeds['Carefull']),
                      transitions={'succeeded':'OppShoot', 'timeout':'OppShoot'})
            
            PreemptiveStateMachine.add('OppShoot',
                      AmbiShootMammoth('Left', p_opponent_side = True),
                      transitions={'succeeded':'PrepareFrescos', 'failed':'PrepareFrescos', 'almostEndGame':'nearlyEndMatch'})

# Go to Frescos entry point
            PreemptiveStateMachine.add('PrepareFrescos',
                      AmbiOmniDirectOrder2(ReverseStickFrescosState.getEntryYellowPoseStatic(), vmax=Robot2014.motionSpeeds['Average']),
                      transitions={'succeeded':'ReverseStickFrescos', 'timeout':'RecoverPrepareStickFrescos'})   #si on y arrive pas, on va au self shoot point
            
            PreemptiveStateMachine.add('RecoverPrepareStickFrescos',
                      AmbiOmniDirectOrder2(Table2014.P_IN_FRONT_START_AREA, vmax=Robot2014.motionSpeeds['Carefull']),
                      transitions={'succeeded':'GoToSFM', 'timeout':'GoToSFT'})
            
#Stick Frescos
            PreemptiveStateMachine.add('ReverseStickFrescos',
                      ReverseStickFrescosState(),
                      transitions={'succeeded':'EmmergencyEscapeFrescos', 'failed':'EmmergencyEscapeFrescos', 'almostEndGame':'nearlyEndMatch' }) # si on y arrive pas on part au point d'urgence de sortie
            
            PreemptiveStateMachine.add('EmmergencyEscapeFrescos',
                      AmbiOmniDirectOrder2(Pose2D(0.200 , 0.400, 0), vmax=Robot2014.motionSpeeds['Carefull']),
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
                      AmbiOmniDirectOrder2(AmbiShootMammoth.getEntryYellowPoseStatic('Right', p_opponent_side = False)),
                      transitions={'succeeded':'SelfShoot', 'timeout':'SelfShoot'}) # si on y arrive pas on passe a la suite

# Shoot Self Mammoth
            PreemptiveStateMachine.add('SelfShoot',
                      AmbiShootMammoth('Right', p_opponent_side = False),
                      transitions={'succeeded':'GoToSFM', 'failed':'GoToSFM', 'almostEndGame':'nearlyEndMatch'})


# Go to Self Fire Mid
            PreemptiveStateMachine.add('GoToSFM',
                      AmbiOmniDirectOrder2Pass(Pose2D(1.200, 0.000, -pi/2), vmax=Robot2014.motionSpeeds['Fast'], vpasse=0.3),
                      transitions={'succeeded':'WaypointBeforeSelfMobileTorch', 'timeout':'EmmergencyPointSFM'}) #Si on y arrive pas, on va au dessus de la torche mobile
            
            
            PreemptiveStateMachine.add('EmmergencyPointSFM',
                      AmbiOmniDirectOrder2Pass(Pose2D(0.600, 0.200, -pi/2), vmax=Robot2014.motionSpeeds['Carefull'], vpasse=0.3),
                      transitions={'succeeded':'WaypointBeforeSelfMobileTorch', 'timeout':'GoToSFB'}) #Si on y arrive pas, on va au Self fire bot

            #faut taper fort dans les fire d'ou le 1.0
            PreemptiveStateMachine.add('WaypointBeforeSelfMobileTorch',
                      AmbiOmniDirectOrder2Pass(Pose2D(1.100, -0.200, -pi/2), vmax=Robot2014.motionSpeeds['Fast'], vpasse=0.3),
                      transitions={'succeeded':'GotoSelfMobileTorch', 'timeout':'GotoSelfMobileTorchFromTop'}) #Si on y arrive pas, on va au dessus de la torche mobile                       

# Alternative push Self Mobile torch from top
            PreemptiveStateMachine.add('GotoSelfMobileTorchFromTop',
                      AmbiOmniDirectOrder2(Pose2D(0.650, -0.150, -2*pi/3), vmax=Robot2014.motionSpeeds['Average']),
                      transitions={'succeeded':'PushSelfMobileTorchFromTop', 'timeout':'PrepareRecalY'})

            PreemptiveStateMachine.add('PushSelfMobileTorchFromTop',
                      AmbiOmniDirectOrder2(Pose2D(0.200, -0.750, -2*pi/3), vmax=Robot2014.motionSpeeds['Fast']),
                      transitions={'succeeded':'GoToSFB', 'timeout':'PrepareRecalY'})
# Push Self Mobile torch            
            PreemptiveStateMachine.add('GotoSelfMobileTorch',
                      AmbiOmniDirectOrder2Pass(Pose2D(0.800, -0.300, 2*pi/3), vmax=Robot2014.motionSpeeds['Fast'], vpasse=0.3),
                      transitions={'succeeded':'PushSelfMobileTorch', 'timeout':'PrepareRecalY'})

            PreemptiveStateMachine.add('PushSelfMobileTorch',
                      AmbiOmniDirectOrder2Pass(Pose2D(0.400, 0.200, 2*pi/3), vmax=Robot2014.motionSpeeds['Fast'], vpasse=0.3),
                      transitions={'succeeded':'WaypointBeforeSmurf', 'timeout':'PrepareRecalY'})
# Go to Smurf Point            
            PreemptiveStateMachine.add('WaypointBeforeSmurf',
                      AmbiOmniDirectOrder2Pass(Pose2D(0.500, -0.100, 0), vmax=Robot2014.motionSpeeds['Fast'], vpasse=0.3),
                      transitions={'succeeded':'SmurfPoint', 'timeout':'PrepareRecalY'})

            PreemptiveStateMachine.add('SmurfPoint',
                      AmbiOmniDirectOrder2Pass(Pose2D(0.500, -0.600, 0), vmax=Robot2014.motionSpeeds['Fast'], vpasse=0.3),
                      transitions={'succeeded':'GoToSFB', 'timeout':'PrepareRecalY'})
                        
# Go to Self Fire Bot
            PreemptiveStateMachine.add('GoToSFB',
                      AmbiOmniDirectOrder2Pass(Pose2D(0.650, -0.600, 0), vmax=Robot2014.motionSpeeds['Fast'], vpasse=0.3),
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
                      AmbiOmniDirectOrder2(Pose2D(-0.200, -0.750, -pi/2), vmax=Robot2014.motionSpeeds['Average']),
                      transitions={'succeeded':'RecalYOpponentTorchBot', 'timeout':'GoToOpponentMobileTorch'})

# Push Opponent Torch Bot
            PreemptiveStateMachine.add('RecalYOpponentTorchBot',
                      AmbiRecalOnBorderYellow("DOWN"),
                      transitions={'recaled':'GoToOpponentFireBotSmurfPoint', 'non-recaled':'GoToOpponentFireBotSmurfPoint','problem':'GoToOpponentEscapePoint'})#'non-recaled':'GoToSFB','problem':'GoToSFB'})  





# Go to Opponent Fire Bot Smurf Point
            PreemptiveStateMachine.add('GoToOpponentFireBotSmurfPoint',
                  AmbiOmniDirectOrder2Pass(Pose2D(-0.300, -0.450, -pi/2), vmax=Robot2014.motionSpeeds['Average'], vpasse=0.3),
                  transitions={'succeeded':'GoToOpponentFireBotWaypoint', 'timeout':'GoToOpponentEscapePoint'})

# Go to Opponent Fire Bot Waypoint
            PreemptiveStateMachine.add('GoToOpponentFireBotWaypoint',
                  AmbiOmniDirectOrder2Pass(Pose2D(-0.700, -0.400, -pi/2), vmax=Robot2014.motionSpeeds['Average'], vpasse=0.3),
                  transitions={'succeeded':'GoToOpponentFireBot', 'timeout':'GoToOpponentEscapePoint'})
            
# Go to Opponent Fire Bot
            PreemptiveStateMachine.add('GoToOpponentFireBot',
                  AmbiOmniDirectOrder2Pass(Pose2D(-0.750, -0.600, 0), vmax=Robot2014.motionSpeeds['Average'], vpasse=0.3),
                  transitions={'succeeded':'PushOpponentFireBot', 'timeout':'GoToOpponentEscapePoint'})
        
# Push Opponent Fire Bot
            PreemptiveStateMachine.add('PushOpponentFireBot',
                  AmbiOmniDirectOrder2Pass(Pose2D(-0.500, -0.600, 0), vmax=Robot2014.motionSpeeds['Fast'], vpasse=0.3),
                  transitions={'succeeded':'GoToOpponentMobileTorch', 'timeout':'GoToOpponentFireMid'})      
            
# Go to Opponent Mobile Torch
            PreemptiveStateMachine.add('GoToOpponentMobileTorch',
                      AmbiOmniDirectOrder2Pass(Pose2D(-0.700, -0.100, -pi/2), vmax=Robot2014.motionSpeeds['Average'], vpasse=0.3),
                      transitions={'succeeded':'GoToOpponentFireMid', 'timeout':'GoToOpponentEscapePoint'})
            
# Go to Opponent Fire Mid
            PreemptiveStateMachine.add('GoToOpponentFireMid',
                      AmbiOmniDirectOrder2Pass(Pose2D(-1.100, -0.200, pi/2), vmax=Robot2014.motionSpeeds['Average'], vpasse=0.3),
                      transitions={'succeeded':'PushOpponentFireMid', 'timeout':'GoToOpponentEscapePoint'})
            
# Push Opponent Fire Mid
            PreemptiveStateMachine.add('PushOpponentFireMid',
                      AmbiOmniDirectOrder2Pass(Pose2D(-1.100, 0.000, pi/2), vmax=Robot2014.motionSpeeds['Fast'], vpasse=0.3),
                      transitions={'succeeded':'FinalLoopOpp', 'timeout':'GoToOpponentEscapePoint'})
            
# Go to Opponent Escape Point
            PreemptiveStateMachine.add('GoToOpponentEscapePoint',
                      AmbiOmniDirectOrder2Pass(Pose2D(0, -0.500, pi/2), vmax=Robot2014.motionSpeeds['Average'], vpasse=0.3),
                      transitions={'succeeded':'GoToOpponentFireMid', 'timeout':'RecalYOpponentTorchBot'})

#Final Loop
            PreemptiveStateMachine.add('FinalLoopOpp',
                      AmbiOmniDirectOrder2Pass(Pose2D(-0.700, -0.600, pi), vmax=Robot2014.motionSpeeds['Fast'], vpasse=0.5),
                      transitions={'succeeded':'FinalLoopMid', 'timeout':'FinalLoopOpp'})
            
            PreemptiveStateMachine.add('FinalLoopMid',
                      AmbiOmniDirectOrder2Pass(Pose2D(0, -0.600, pi/2), vmax=Robot2014.motionSpeeds['Fast'], vpasse=0.5),
                      transitions={'succeeded':'FinalLoopSelf', 'timeout':'FinalLoopOpp'})
            
            PreemptiveStateMachine.add('FinalLoopSelf',
                      AmbiOmniDirectOrder2Pass(Pose2D(0.700, -0.600, 0), vmax=Robot2014.motionSpeeds['Fast'], vpasse=0.5),
                      transitions={'succeeded':'FinalLoopOpp', 'timeout':'FinalLoopMid'})            

    
####################################################################################################################
## End of Rush
####################################################################################################################
