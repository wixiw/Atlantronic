#!/usr/bin/env python

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
                      transitions={'succeeded':'GoToYFT', 'timeout':'RetryEscapeStartArea'})
            self.setInitialState('EscapeStartArea')
            
            PreemptiveStateMachine.add('RetryEscapeStartArea',
                      AmbiOmniDirectOrder2Pass(Pose2D(Table2014.P_YOU_HOU.x, Table2014.P_YOU_HOU.y -0.050, Table2014.P_YOU_HOU.theta) ,vpasse=-1),
                      transitions={'succeeded':'GoToYFT', 'timeout':'GoToYFT'})
            
            
# Go to Yellow Fire Top
            PreemptiveStateMachine.add('GoToYFT',
                      AmbiOmniDirectOrder2Pass( Pose2D(0.650 + Robot2014.FRONT_SIDE.x, 0.300, -5*pi/6),vpasse=-1),
                      transitions={'succeeded':'PrepareFrescos', 'timeout':'PrepareFrescos'})

#GOTO StickFrescos


# Go to old Gay Camping point, align for ShootStates
#            PreemptiveStateMachine.add('PrepareFrescos',
#                      AmbiOmniDirectOrder2Pass(Pose2D(0.300, 0.400, -5*pi/6),vpasse=-1),
#                      transitions={'succeeded':'StickFrescos', 'timeout':'problem'})   

# Go to Gay Camping point
            PreemptiveStateMachine.add('PrepareFrescos',
                      AmbiOmniDirectOrder2(Pose2D(-0.160, 0.480, -pi/4)),
                      transitions={'succeeded':'WaitCamping', 'timeout':'WaitCamping'})   

# Wait camping     
            PreemptiveStateMachine.add('WaitCamping',
                      WaiterState(2.0),
                      transitions={'timeout':'StickFrescos'})
            
#Go to stick to frescos entry point not requiered as it is the same as DoubleTargetShootState
#Stick Frescos
            PreemptiveStateMachine.add('StickFrescos',
                      StickFrescosState(),
                      transitions={'endFrescos':'DoubleTargetShoot', 'problem':'askSelector'})

# Shoot
            PreemptiveStateMachine.add('DoubleTargetShoot',
                      DoubleTargetShootState(),
                      transitions={'endShoot':'GoToAlmostCentralHeat', 'problem':'askSelector'})

# Go to Almost Central Heat
            PreemptiveStateMachine.add('GoToAlmostCentralHeat',
                      AmbiOmniDirectOrder2(Pose2D(0.000, 0.100, -pi/2)),
                      transitions={'succeeded':'DropYFT', 'timeout':'motionBlocked'})

# Drop  Yellow Fire Top     
            PreemptiveStateMachine.add('DropYFT',
                      WaiterState(1.5),
                      transitions={'timeout':'EscapeAlmostCentralHeat'})
            
# Escape from Almost Central Heat
            PreemptiveStateMachine.add('EscapeAlmostCentralHeat',
                      AmbiOmniDirectOrder2(Pose2D(0.220, 0.150, -pi/2)),
                      transitions={'succeeded':'GoToYellowMobileTorch', 'timeout':'motionBlocked'})



#Raccourci
                        
# Go to Yellow Mobile Torch
            PreemptiveStateMachine.add('GoToYellowMobileTorch',
                      AmbiOmniDirectOrder2(Pose2D(Table2014.P_YELLOW_MOBILE_TORCH.x - Robot2014.FRONT_SIDE.x - Table2014.MOBILE_TORCH_RADIUS, Table2014.P_YELLOW_MOBILE_TORCH.y - Table2014.MOBILE_TORCH_RADIUS, 0)),
                      transitions={'succeeded':'pause', 'timeout':'motionBlocked'})

            PreemptiveStateMachine.add('pause',
                      WaiterState(3.0),
                      transitions={'timeout':'ApproachYellowMobileTorch'})

# Doit degager avec le cpoint     
# Approach  Yellow Mobile Torch     
            PreemptiveStateMachine.add('ApproachYellowMobileTorch',
                      AmbiOmniDirectOrder2(Pose2D(Table2014.P_YELLOW_MOBILE_TORCH.x, Table2014.P_YELLOW_MOBILE_TORCH.y, pi/6)),
                      transitions={'succeeded':'PushEmptyYellowMobileTorch', 'timeout':'motionBlocked'})

# Turn around Yellow Mobile Torch
            PreemptiveStateMachine.add('TurnEmptyYellowMobileTorch',
                      AmbiOmniDirectOrder2(Pose2D(0.600, 0.000, pi/6)),
                     #AmbiOmniDirectOrder_cpoint(0.100, 0.000, 0.0, 
                     #                           0.600,-0.100,-5*pi/6),
                      transitions={'succeeded':'PushEmptyYellowMobileTorch', 'timeout':'motionBlocked'})


                        
# Push Empty Yellow Mobile Torch
            PreemptiveStateMachine.add('PushEmptyYellowMobileTorch',
                      AmbiOmniDirectOrder2(Pose2D(1.225, 0.525, pi/3)),
                      transitions={'succeeded':'ActionFireYellowMobileTorch', 'timeout':'motionBlocked'})


# Action Fire Yellow Mobile Torch     
            PreemptiveStateMachine.add('ActionFireYellowMobileTorch',
                      WaiterState(3.0),
                      transitions={'timeout':'PrepareRecalY'})





# Escape Empty Yellow Mobile Torch 
            PreemptiveStateMachine.add('EscapeEmptyYellowMobileTorch',
                      AmbiOmniDirectOrder2(Pose2D(0.200, -0.800, -pi/2)),
                      transitions={'succeeded':'GoToYellowTorchBot', 'timeout':'motionBlocked'})

# Go to Yellow Torch Bot

            PreemptiveStateMachine.add('GoToYellowTorchBot',
                      AmbiOmniDirectOrder2(Pose2D(0.200, -1.000 + Robot2014.FRONT_SIDE.x, -pi/2)),
                      transitions={'succeeded':'ActionFireYellowTorchBot', 'timeout':'motionBlocked'})
 
# Action Fires Yellow Torch Bot     
            PreemptiveStateMachine.add('ActionFireYellowTorchBot',
                      WaiterState(1.5),
                      transitions={'timeout':'GoToYFB'})

# Escape Yellow Torch Bot

# Go to Yellow Fire Bot
            PreemptiveStateMachine.add('GoToYFB',
                      AmbiOmniDirectOrder2(Pose2D(0.600 - Robot2014.FRONT_SIDE.x,-0.600,0), vmax = 1.0),
                      transitions={'succeeded':'PickYFB', 'timeout':'motionBlocked'})

# Pick Yellow Fire Bot            
            PreemptiveStateMachine.add('PickYFB',
                      WaiterState(1.5),
                      transitions={'timeout':'PrepareRecalY'})









#Saut de strat pour tester les ordres de recalage.


            PreemptiveStateMachine.add('PrepareRecalY',
                      AmbiOmniDirectOrder2(Pose2D(0.800,0.500,pi/2)),
                      transitions={'succeeded':'RecalY', 'timeout':'motionBlocked'})
            
            PreemptiveStateMachine.add('RecalY',
                      AmbiRecalOnBorderYellow("FRUITBASKET",Data.color),
                      transitions={'recaled':'EscapeRecalY', 'non-recaled':'askSelector','problem':'motionBlocked'})  

            PreemptiveStateMachine.add('EscapeRecalY',
                      AmbiOmniDirectOrder2(Pose2D(1.200,0.400,pi/4)),
                      transitions={'succeeded':'PrepareRecalX', 'timeout':'motionBlocked'})
             
            PreemptiveStateMachine.add('PrepareRecalX',
                      AmbiOmniDirectOrder2(Pose2D(1.350,0.550,0), vmax=0.3),
                      transitions={'succeeded':'RecalX', 'timeout':'motionBlocked'})
            
            PreemptiveStateMachine.add('RecalX',
                      AmbiRecalOnBorderYellow("RIGHT",Data.color),
                      transitions={'recaled':'PrepareUnloadFireInStartArea', 'non-recaled':'askSelector','problem':'motionBlocked'})

            
# Enter Start Area
            PreemptiveStateMachine.add('PrepareUnloadFireInStartArea',
                      AmbiOmniDirectOrder2(Pose2D(1.225, 0.525, pi/3), vmax=0.3),
                      transitions={'succeeded':'DropFiresStartArea', 'timeout':'motionBlocked'})

# Drop Fires in stock in Start area       
            PreemptiveStateMachine.add('DropFiresStartArea',
                      WaiterState(2.5),
                      transitions={'timeout':'ExitStartArea'})

# Exit Start Area
            PreemptiveStateMachine.add('ExitStartArea',
                      AmbiOmniDirectOrder2(Table2014.P_IN_FRONT_START_AREA),
                      transitions={'succeeded':'Timer', 'timeout':'motionBlocked'})
            
# Timer             
            PreemptiveStateMachine.add('Timer',
                      WaiterState(5),
                      transitions={'timeout':'GoToYFM'})
                                                           
# Go to Yellow Fire Mid
            PreemptiveStateMachine.add('GoToYFM',
                      AmbiOmniDirectOrder2(Pose2D(1.100, 0.000 - Robot2014.FRONT_SIDE.x, -pi/2)),
                      transitions={'succeeded':'PickYFM', 'timeout':'motionBlocked'})

# Pick Yellow Fire Mid            
            PreemptiveStateMachine.add('PickYFM',
                      WaiterState(1.5),
                      transitions={'timeout':'GoToYellowTorchMid'})
            
# Go to Yellow Torch Mid
            PreemptiveStateMachine.add('GoToYellowTorchMid',
                      AmbiOmniDirectOrder2(Pose2D(1.500 - Robot2014.FRONT_SIDE.x, 0.200, 0)),
                      transitions={'succeeded':'ActionFireYellowTorchMid', 'timeout':'motionBlocked'})
 
# Action Fires Yellow Torch Mid     
            PreemptiveStateMachine.add('ActionFireYellowTorchMid',
                      WaiterState(1.5),
                      transitions={'timeout':'TreeAvoidance'})

# Tree avoidance
            PreemptiveStateMachine.add('TreeAvoidance',
                      AmbiOmniDirectOrder2Pass(Pose2D(1.200, -0.300, 0), vpasse=1.0),
                      transitions={'succeeded':'GoToHeatBot', 'timeout':'motionBlocked'})
            
# Go to Heat Bot
            PreemptiveStateMachine.add('GoToHeatBot',
                      AmbiOmniDirectOrder2(Pose2D(1.300 + Robot2014.RIGHT_SIDE.x, -0.750 - Robot2014.FRONT_SIDE.x, -pi/4)),
                      transitions={'succeeded':'DropFiresHeatBot', 'timeout':'motionBlocked'})
            
# Drop Fires in stock in Heat bot        
            PreemptiveStateMachine.add('DropFiresHeatBot',
                      WaiterState(5.0),
                      transitions={'timeout':'PrepareRecalBotY'})

#Recalage bot
            PreemptiveStateMachine.add('PrepareRecalBotY',
                      AmbiOmniDirectOrder2(Pose2D(1.500 - Robot2014.LEFT_SIDE.y - Table2014.HEAT_BOT_RADIUS, -0.800, -pi/2)),
                      transitions={'succeeded':'RecalBotY', 'timeout':'motionBlocked'})
            
            PreemptiveStateMachine.add('RecalBotY',
                      AmbiRecalOnBorderYellow("DOWN",Data.color),
                      transitions={'recaled':'EscapeRecalBotY', 'non-recaled':'askSelector','problem':'motionBlocked'})  

            PreemptiveStateMachine.add('EscapeRecalBotY',
                      AmbiOmniDirectOrder2(Pose2D(1.500 - Robot2014.LEFT_SIDE.y - Table2014.HEAT_BOT_RADIUS, -0.600, 0)),
                      transitions={'succeeded':'PrepareRecalBotX', 'timeout':'motionBlocked'})
             
            PreemptiveStateMachine.add('PrepareRecalBotX',
                      AmbiOmniDirectOrder2(Pose2D(1.100, -1.000 - Robot2014.RIGHT_SIDE.y + Table2014.HEAT_BOT_RADIUS, 0), vmax=0.3),
                      transitions={'succeeded':'RecalBotX', 'timeout':'motionBlocked'})
            
            PreemptiveStateMachine.add('RecalBotX',
                      AmbiRecalOnBorderYellow("RIGHT",Data.color),
                      transitions={'recaled':'WaitBeforeNext', 'non-recaled':'askSelector','problem':'motionBlocked'})
            
# Useless timer            
            PreemptiveStateMachine.add('WaitBeforeNext',
                      WaiterState(1.5),
                      transitions={'timeout':'endOpening'})
