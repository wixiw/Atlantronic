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
                                             EndMatchPreempter(-5.0),
                                             transitions={'endMatch':'nearlyEndMatch'})
            
            PreemptiveStateMachine.add('EscapeStartArea',
                      AmbiOmniDirectOrder2Pass(Table2014.P_YOU_HOU,vpasse=-1),
                      transitions={'succeeded':'GoToSFT', 'timeout':'RetryEscapeStartArea'})
            self.setInitialState('EscapeStartArea')
            
            PreemptiveStateMachine.add('RetryEscapeStartArea',
                      AmbiOmniDirectOrder2Pass(Pose2D(Table2014.P_YOU_HOU.x, Table2014.P_YOU_HOU.y -0.050, Table2014.P_YOU_HOU.theta) ,vpasse=-1),
                      transitions={'succeeded':'GoToSFT', 'timeout':'EscapeStartArea'})
            
                
# Go to Self Fire Top
            PreemptiveStateMachine.add('GoToSFT',
                      AmbiOmniDirectOrder2Pass(Pose2D(0.650 + Robot2014.FRONT_SIDE.x, 0.300, -5*pi/6),vpasse=-1),
                      transitions={'succeeded':'GotoOpponentShootPoint', 'timeout':'EscapeStartArea'}) #si on y arrive pas on repart au depart et on recommence

## Go through Opponent Shoot Point
#            PreemptiveStateMachine.add('GoThroughOpponentShootPoint',
#                      AmbiOmniDirectOrder2Pass(Pose2D(-0.600, 0.300, -5*pi/6),vpasse=-1),
#                      transitions={'succeeded':'GotoOpponentShootPoint', 'timeout':'PrepareFrescos'}) #si on y arrive pas, tant pis on passe l'action on va coller
#            
# Go to Opponent Shoot Point
            PreemptiveStateMachine.add('GotoOpponentShootPoint',
                      AmbiOmniDirectOrder2(AmbiShootMammoth.getEntryYellowPoseStatic('Left', p_opponent_side = True)),
                      transitions={'succeeded':'WaitOppShoot', 'timeout':'PrepareFrescos'}) #si on y arrive pas, tant pis on passe l'action on va coller

#Simulate opp shoot
            PreemptiveStateMachine.add('WaitOppShoot',
                      WaiterState(1.0),
                      transitions={'timeout':'PrepareFrescos'})
            
# Shoot Opponent Mammoth
#            PreemptiveStateMachine.add('TargetShoot',
#                      AmbiShootMammoth('Left', p_opponent_side = True),
#                      transitions={'succeeded':'PrepareFrescos', 'failed':'nearlyEndMatch', 'almostEndGame':'nearlyEndMatch'})

## Go to Gay Camping point
#            PreemptiveStateMachine.add('GotoGayCampingPoint',
#                      AmbiOmniDirectOrder2(AmbiShootMammoth.getEntryYellowPoseStatic('Left', p_opponent_side = True)),
#                      transitions={'succeeded':'PrepareFrescos', 'timeout':'nearlyEndMatch'})   

# Go to Frescos entry point
            PreemptiveStateMachine.add('PrepareFrescos',
                      AmbiOmniDirectOrder2(StickFrescosState.getEntryYellowPoseStatic()),
                      transitions={'succeeded':'StickFrescos', 'timeout':'GotoSelfShootPoint'})   #si on y arrive pas, on va au self shoot point
            
#Stick Frescos
            PreemptiveStateMachine.add('StickFrescos',
                      StickFrescosState(),
                      transitions={'succeeded':'GotoSelfShootPoint', 'failed':'EmmergencyEscapeFrescos', 'almostEndGame':'nearlyEndMatch' }) # si on y arrive pas on part au point d'urgence de sortie
            
            PreemptiveStateMachine.add('EmmergencyEscapeFrescos',
                      AmbiOmniDirectOrder2(Pose2D(0 , 0.300, pi/2)),
                      transitions={'succeeded':'GotoSelfShootPoint', 'timeout':'GotoSelfShootPoint' })
            
##DEBUG WILLY POUR HOMOLOGATION EVITEMENT
#            PreemptiveStateMachine.add('ReturnInStartArea',
#                      AmbiOmniDirectOrder2(Table2014.P_YOU_HOU),
#                      transitions={'succeeded':'CyclePoint', 'timeout':'CyclePoint'})
#            
#            #DEBUG WILLY POUR HOMOLOGATION EVITEMENT
#            PreemptiveStateMachine.add('CyclePoint',
#                      AmbiOmniDirectOrder2(Pose2D(0.600, -0.600, 0)),
#                      transitions={'succeeded':'ReturnInStartArea', 'timeout':'ReturnInStartArea'})

# Go to Self Shoot Point
            PreemptiveStateMachine.add('GotoSelfShootPoint',
                      AmbiOmniDirectOrder2(AmbiShootMammoth.getEntryYellowPoseStatic('Left', p_opponent_side = False)),
                      transitions={'succeeded':'WaitSelfShoot', 'timeout':'GoToSFM'}) # si on y arrive pas on passe a la suite

#Simulate self shoot
            PreemptiveStateMachine.add('WaitSelfShoot',
                      WaiterState(1.0),
                      transitions={'timeout':'GoToSFM'})
# Shoot Self Mammoth
#            PreemptiveStateMachine.add('TargetShoot',
#                      AmbiShootMammoth('Right', p_opponent_side = False),
#                      transitions={'succeeded':'GoToSFM', 'failed':'nearlyEndMatch', 'almostEndGame':'nearlyEndMatch'})


# Go to Self Fire Mid
            PreemptiveStateMachine.add('GoToSFM',
                      AmbiOmniDirectOrder2Pass(Pose2D(1.200, 0.000, -pi/2), vmax=0.5, vpasse=0.3),
                      transitions={'succeeded':'WaypointBeforeSmurf', 'timeout':'EmmergencyPointSFM'}) #Si on y arrive pas, on va au dessus de la torche mobile
            
            
            PreemptiveStateMachine.add('EmmergencyPointSFM',
                      AmbiOmniDirectOrder2Pass(Pose2D(0.600, 0.200, -pi/2), vmax=0.5, vpasse=0.3),
                      transitions={'succeeded':'WaypointBeforeSmurf', 'timeout':'GoToSFB'}) #Si on y arrive pas, on va au Self fire bot

            PreemptiveStateMachine.add('WaypointBeforeSmurf',
                      AmbiOmniDirectOrder2Pass(Pose2D(1.100, -0.200, -pi/2), vmax=1.0, vpasse=0.3),
                      transitions={'succeeded':'GoToSmurfPoint', 'timeout':'GoToSmurfPoint'}) #Si on y arrive pas, on va au dessus de la torche mobile                       

# Go to Smurf Point to go to Self Fire Bot
            PreemptiveStateMachine.add('GoToSmurfPoint',
                      AmbiOmniDirectOrder2Pass(Pose2D(0.300, -0.300, -pi/2), vmax=0.5, vpasse=0.3),
                      transitions={'succeeded':'GoToSFB', 'timeout':'PrepareRecalY'}) #Si on y arrive pas, on va au dessus de la torche mobile

# Go to Self Fire Bot
            PreemptiveStateMachine.add('GoToSFB',
                      AmbiOmniDirectOrder2Pass(Pose2D(0.600, -0.600, 0), vmax=1.0, vpasse=0.3),
                      transitions={'succeeded':'PrepareRecalY', 'timeout':'EmergencyMiddlePoint'}) #Si on y arrive pas, on va au Emergency Middle Point

            PreemptiveStateMachine.add('EmergencyMiddlePoint',
                      AmbiOmniDirectOrder2(Pose2D(1.200, 0.200, -pi/2), vmax=0.5),
                      transitions={'succeeded':'PrepareRecalY', 'timeout':'GotoOpponentShootPoint'}) #Si on y arrive pas, on retourne au opponent shoot point
            
#Ordres de recalage Top

            PreemptiveStateMachine.add('PrepareRecalY',
                      AmbiOmniDirectOrder2(Pose2D(0.800,0.500,pi/2), vmax=0.5),
                      transitions={'succeeded':'RecalY', 'timeout':'motionBlocked'})
            
            PreemptiveStateMachine.add('RecalY',
                      AmbiRecalOnBorderYellow("FRUITBASKET",Data.color),
                      transitions={'recaled':'EscapeRecalY', # 'non-recaled':'askSelector','problem':'motionBlocked'})
                                   'non-recaled':'PrepareRecalY','problem':'PrepareRecalY'})  

            PreemptiveStateMachine.add('EscapeRecalY',
                      AmbiOmniDirectOrder2(Pose2D(1.200,0.400,pi/4), vmax=0.5),
                      transitions={'succeeded':'GoToSelfTorchMid', 'timeout':'PrepareRecalY'})
             
#            PreemptiveStateMachine.add('PrepareRecalX',
#                      AmbiOmniDirectOrder2(Pose2D(1.350,0.550,0), vmax=0.3),
#                      transitions={'succeeded':'RecalX', 'timeout':'PrepareRecalY'})
#            
#            PreemptiveStateMachine.add('RecalX',
#                      AmbiRecalOnBorderYellow("RIGHT",Data.color),
#                      transitions={'recaled':'GoToSelfTorchMid', 'non-recaled':'PrepareRecalY','problem':'PrepareRecalY'})

# Go to Self Torch Mid
            PreemptiveStateMachine.add('GoToSelfTorchMid',
                      AmbiOmniDirectOrder2(Pose2D(1.285, 0.200, 0), vmax=0.5),
                      transitions={'succeeded':'RecalXSelfTorchMid', 'timeout':'PrepareRecalY'})
# Push Self Torch Mid
            PreemptiveStateMachine.add('RecalXSelfTorchMid',
                      AmbiRecalOnBorderYellow("RIGHT",Data.color),
                      transitions={'recaled':'WaipointFireBot', 'non-recaled':'PrepareRecalY','problem':'PrepareRecalY'})

# Go to WaipointFire Bot
            PreemptiveStateMachine.add('WaipointFireBot',
                      AmbiOmniDirectOrder2(Pose2D(0.600, -0.600, -pi/4), vmax=0.5),
                      transitions={'succeeded':'GoToOpponentTorchBot', 'timeout':'GoToSFB'})
            
# Go to Opponent Torch Bot
            PreemptiveStateMachine.add('GoToOpponentTorchBot',
                      AmbiOmniDirectOrder2(Pose2D(-0.200, -0.885, -pi/2), vmax=0.5),
                      transitions={'succeeded':'RecalYOpponentTorchBot', 'timeout':'GoToSFB'})

# Push Opponent Torch Bot
            PreemptiveStateMachine.add('RecalYOpponentTorchBot',
                      AmbiRecalOnBorderYellow("DOWN",Data.color),
                      transitions={'recaled':'GoToOpponentMobileTorch', 'non-recaled':'GoToSFB','problem':'GoToSFB'})  

# Go to Opponent Mobile Torch
            PreemptiveStateMachine.add('GoToOpponentMobileTorch',
                      AmbiOmniDirectOrder2Pass(Pose2D(-0.500, -0.100, -pi/2), vmax=0.5, vpasse=0.3),
                      transitions={'succeeded':'GoToOpponentFireMid', 'timeout':'GoToOpponentEscapePoint'})
            
# Go to Opponent Escape Point
            PreemptiveStateMachine.add('GoToOpponentEscapePoint',
                      AmbiOmniDirectOrder2Pass(Pose2D(0, -0.500, pi/2), vmax=0.5, vpasse=0.3),
                      transitions={'succeeded':'GoToSFB', 'timeout':'GoToSFB'})
            
# Go to Opponent Fire Mid
            PreemptiveStateMachine.add('GoToOpponentFireMid',
                      AmbiOmniDirectOrder2Pass(Pose2D(-1.100, -0.200, pi/2), vmax=0.5, vpasse=0.3),
                      transitions={'succeeded':'PushOpponentFireMid', 'timeout':'GoToSFB'})
            
# Push Opponent Fire Mid
            PreemptiveStateMachine.add('PushOpponentFireMid',
                      AmbiOmniDirectOrder2Pass(Pose2D(-1.100, 0.000, pi/2), vmax=0.5, vpasse=0.3),
                      transitions={'succeeded':'GoToOpponentFireBot', 'timeout':'GoToSFB'})
            
# Go to Opponent Fire Bot
            PreemptiveStateMachine.add('GoToOpponentFireBot',
                  AmbiOmniDirectOrder2Pass(Pose2D(-0.700, -0.600, 0), vmax=0.5, vpasse=0.3),
                  transitions={'succeeded':'PushOpponentFireBot', 'timeout':'GoToSFB'})
        
# Push Opponent Fire Bot
            PreemptiveStateMachine.add('PushOpponentFireBot',
                  AmbiOmniDirectOrder2Pass(Pose2D(-0.500, -0.600, 0), vmax=0.5, vpasse=0.3),
                  transitions={'succeeded':'GoToSFB', 'timeout':'GoToSFB'})       
        

## Go to Almost Central Heat
#            PreemptiveStateMachine.add('GoToAlmostCentralHeat',
#                      AmbiOmniDirectOrder2(Pose2D(0.000, 0.100, -pi/2)),
#                      transitions={'succeeded':'DropYFT', 'timeout':'motionBlocked'})
#
## Drop  Yellow Fire Top     
#            PreemptiveStateMachine.add('DropYFT',
#                      WaiterState(1.5),
#                      transitions={'timeout':'EscapeAlmostCentralHeat'})
#            
## Escape from Almost Central Heat
#            PreemptiveStateMachine.add('EscapeAlmostCentralHeat',
#                      AmbiOmniDirectOrder2(Pose2D(0.220, 0.150, -pi/2)),
#                      transitions={'succeeded':'endOpening', 'timeout':'motionBlocked'})

####################################################################################################################
## End of Rush
####################################################################################################################
#
## Following stuff shall go to dedicated LocalStrategicActions in the MiddleGame 
#
#
#
##Raccourci
#                        
## Go to Yellow Mobile Torch
#            PreemptiveStateMachine.add('GoToYellowMobileTorch',
#                      AmbiOmniDirectOrder2(Pose2D(Table2014.P_YELLOW_MOBILE_TORCH.x - Robot2014.FRONT_SIDE.x - Table2014.MOBILE_TORCH_RADIUS, Table2014.P_YELLOW_MOBILE_TORCH.y - Table2014.MOBILE_TORCH_RADIUS, 0)),
#                      transitions={'succeeded':'pause', 'timeout':'motionBlocked'})
#
#            PreemptiveStateMachine.add('pause',
#                      WaiterState(3.0),
#                      transitions={'timeout':'ApproachYellowMobileTorch'})
#
## Doit degager avec le cpoint     
## Approach  Yellow Mobile Torch     
#            PreemptiveStateMachine.add('ApproachYellowMobileTorch',
#                      AmbiOmniDirectOrder2(Pose2D(Table2014.P_YELLOW_MOBILE_TORCH.x, Table2014.P_YELLOW_MOBILE_TORCH.y, pi/6)),
#                      transitions={'succeeded':'PushEmptyYellowMobileTorch', 'timeout':'motionBlocked'})
#
## Turn around Yellow Mobile Torch
#            PreemptiveStateMachine.add('TurnEmptyYellowMobileTorch',
#                      AmbiOmniDirectOrder2(Pose2D(0.600, 0.000, pi/6)),
#                     #AmbiOmniDirectOrder_cpoint(0.100, 0.000, 0.0, 
#                     #                           0.600,-0.100,-5*pi/6),
#                      transitions={'succeeded':'PushEmptyYellowMobileTorch', 'timeout':'motionBlocked'})
#
#
#                        
## Push Empty Yellow Mobile Torch
#            PreemptiveStateMachine.add('PushEmptyYellowMobileTorch',
#                      AmbiOmniDirectOrder2(Pose2D(1.225, 0.525, pi/3)),
#                      transitions={'succeeded':'ActionFireYellowMobileTorch', 'timeout':'motionBlocked'})
#
#
## Action Fire Yellow Mobile Torch     
#            PreemptiveStateMachine.add('ActionFireYellowMobileTorch',
#                      WaiterState(3.0),
#                      transitions={'timeout':'PrepareRecalY'})
#
#
#
#
#
## Escape Empty Yellow Mobile Torch 
#            PreemptiveStateMachine.add('EscapeEmptyYellowMobileTorch',
#                      AmbiOmniDirectOrder2(Pose2D(0.200, -0.800, -pi/2)),
#                      transitions={'succeeded':'GoToYellowTorchBot', 'timeout':'motionBlocked'})
#
## Go to Yellow Torch Bot
#
#            PreemptiveStateMachine.add('GoToYellowTorchBot',
#                      AmbiOmniDirectOrder2(Pose2D(0.200, -1.000 + Robot2014.FRONT_SIDE.x, -pi/2)),
#                      transitions={'succeeded':'ActionFireYellowTorchBot', 'timeout':'motionBlocked'})
# 
## Action Fires Yellow Torch Bot     
#            PreemptiveStateMachine.add('ActionFireYellowTorchBot',
#                      WaiterState(1.5),
#                      transitions={'timeout':'GoToYFB'})
#
## Escape Yellow Torch Bot
#
## Go to Yellow Fire Bot
#            PreemptiveStateMachine.add('GoToYFB',
#                      AmbiOmniDirectOrder2(Pose2D(0.600 - Robot2014.FRONT_SIDE.x,-0.600,0), vmax = 1.0),
#                      transitions={'succeeded':'PickYFB', 'timeout':'motionBlocked'})
#
## Pick Yellow Fire Bot            
#            PreemptiveStateMachine.add('PickYFB',
#                      WaiterState(1.5),
#                      transitions={'timeout':'PrepareRecalY'})
#
####################################################################################################################
#
#
#
#
#
#
#

#
#            
## Enter Start Area
#            PreemptiveStateMachine.add('PrepareUnloadFireInStartArea',
#                      AmbiOmniDirectOrder2(Pose2D(1.225, 0.525, pi/3), vmax=0.3),
#                      transitions={'succeeded':'DropFiresStartArea', 'timeout':'motionBlocked'})
#
## Drop Fires in stock in Start area       
#            PreemptiveStateMachine.add('DropFiresStartArea',
#                      WaiterState(2.5),
#                      transitions={'timeout':'ExitStartArea'})
#
## Exit Start Area
#            PreemptiveStateMachine.add('ExitStartArea',
#                      AmbiOmniDirectOrder2(Table2014.P_IN_FRONT_START_AREA),
#                      transitions={'succeeded':'Timer', 'timeout':'motionBlocked'})
#            
## Timer             
#            PreemptiveStateMachine.add('Timer',
#                      WaiterState(5),
#                      transitions={'timeout':'GoToYFM'})
#                                                           
## Go to Yellow Fire Mid
#            PreemptiveStateMachine.add('GoToYFM',
#                      AmbiOmniDirectOrder2(Pose2D(1.100, 0.000 - Robot2014.FRONT_SIDE.x, -pi/2)),
#                      transitions={'succeeded':'PickYFM', 'timeout':'motionBlocked'})
#
## Pick Yellow Fire Mid            
#            PreemptiveStateMachine.add('PickYFM',
#                      WaiterState(1.5),
#                      transitions={'timeout':'GoToYellowTorchMid'})
#            
## Go to Yellow Torch Mid
#            PreemptiveStateMachine.add('GoToYellowTorchMid',
#                      AmbiOmniDirectOrder2(Pose2D(1.500 - Robot2014.FRONT_SIDE.x, 0.200, 0)),
#                      transitions={'succeeded':'ActionFireYellowTorchMid', 'timeout':'motionBlocked'})
# 
## Action Fires Yellow Torch Mid     
#            PreemptiveStateMachine.add('ActionFireYellowTorchMid',
#                      WaiterState(1.5),
#                      transitions={'timeout':'TreeAvoidance'})
#
## Tree avoidance
#            PreemptiveStateMachine.add('TreeAvoidance',
#                      AmbiOmniDirectOrder2Pass(Pose2D(1.200, -0.300, 0), vpasse=1.0),
#                      transitions={'succeeded':'GoToHeatBot', 'timeout':'motionBlocked'})
#            
## Go to Heat Bot
#            PreemptiveStateMachine.add('GoToHeatBot',
#                      AmbiOmniDirectOrder2(Pose2D(1.300 + Robot2014.RIGHT_SIDE.x, -0.750 - Robot2014.FRONT_SIDE.x, -pi/4)),
#                      transitions={'succeeded':'DropFiresHeatBot', 'timeout':'motionBlocked'})
#            
## Drop Fires in stock in Heat bot        
#            PreemptiveStateMachine.add('DropFiresHeatBot',
#                      WaiterState(5.0),
#                      transitions={'timeout':'PrepareRecalBotY'})
#
##Recalage bot
#            PreemptiveStateMachine.add('PrepareRecalBotY',
#                      AmbiOmniDirectOrder2(Pose2D(1.500 - Robot2014.LEFT_SIDE.y - Table2014.HEAT_BOT_RADIUS, -0.800, -pi/2)),
#                      transitions={'succeeded':'RecalBotY', 'timeout':'motionBlocked'})
#            
#            PreemptiveStateMachine.add('RecalBotY',
#                      AmbiRecalOnBorderYellow("DOWN",Data.color),
#                      transitions={'recaled':'EscapeRecalBotY', 'non-recaled':'askSelector','problem':'motionBlocked'})  
#
#            PreemptiveStateMachine.add('EscapeRecalBotY',
#                      AmbiOmniDirectOrder2(Pose2D(1.500 - Robot2014.LEFT_SIDE.y - Table2014.HEAT_BOT_RADIUS, -0.600, 0)),
#                      transitions={'succeeded':'PrepareRecalBotX', 'timeout':'motionBlocked'})
#             
#            PreemptiveStateMachine.add('PrepareRecalBotX',
#                      AmbiOmniDirectOrder2(Pose2D(1.100, -1.000 - Robot2014.RIGHT_SIDE.y + Table2014.HEAT_BOT_RADIUS, 0), vmax=0.3),
#                      transitions={'succeeded':'RecalBotX', 'timeout':'motionBlocked'})
#            
#            PreemptiveStateMachine.add('RecalBotX',
#                      AmbiRecalOnBorderYellow("RIGHT",Data.color),
#                      transitions={'recaled':'WaitBeforeNext', 'non-recaled':'askSelector','problem':'motionBlocked'})
#            
## Useless timer            
#            PreemptiveStateMachine.add('WaitBeforeNext',
#                      WaiterState(1.5),
#                      transitions={'timeout':'endOpening'})
