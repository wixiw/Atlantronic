#!/usr/bin/env python

#libraries for ROS
import roslib; roslib.load_manifest('arp_master')
from arp_master import *
from arp_master.strat_2014 import *

class Opening(PreemptiveStateMachine):
    def __init__(self):
        PreemptiveStateMachine.__init__(self, outcomes=['endOpening', 'problem'])
        with self:
            PreemptiveStateMachine.addPreemptive('EndMatchPreemption',
                                             EndMatchPreempter(-5.0),
                                             transitions={'endMatch':'problem'})
            
            PreemptiveStateMachine.add('EscapeStartArea',
                      AmbiOmniDirectOrder2( Pose2D(1.100, 0.400, -5*pi/6)),
                      transitions={'succeeded':'GoToYFT', 'timeout':'problem'})

            self.setInitialState('EscapeStartArea')
            
# Go to Yellow Fire Top
            PreemptiveStateMachine.add('GoToYFT',
                      AmbiOmniDirectOrder2Pass( Pose2D(0.600 + Robot2014.FRONT_SIDE.x, 0.400, -5*pi/6), vpasse=1),
                      transitions={'succeeded':'PrepareFrescos', 'timeout':'problem'})

# Go to Gay Camping point, align for ShootStates
            PreemptiveStateMachine.add('PrepareFrescos',
                      AmbiOmniDirectOrder2(Pose2D(0.300, 0.400, -5*pi/6)),
                      transitions={'succeeded':'StickFrescos', 'timeout':'problem'})

#Go to stick to frescos entry point not requiered as it is the same as DoubleTargetShootState     
#Stick Frescos
            PreemptiveStateMachine.add('StickFrescos',
                      StickFrescosState(),
                      transitions={'endFrescos':'GoToAlmostCentralHeat', 'problem':'problem'})
            
# Go to Almost Central Heat
            PreemptiveStateMachine.add('GoToAlmostCentralHeat',
                      AmbiOmniDirectOrder2(Pose2D(0.000, 0.100, -pi/2)),
                      transitions={'succeeded':'DropYFT', 'timeout':'problem'})

# Drop  Yellow Fire Top     
            PreemptiveStateMachine.add('DropYFT',
                      WaiterState(1.5),
                      transitions={'timeout':'EscapeAlmostCentralHeat'})
            
# Escape from Almost Central Heat
            PreemptiveStateMachine.add('EscapeAlmostCentralHeat',
                      AmbiOmniDirectOrder2(Pose2D(0.220, 0.150, -pi/2)),
                      transitions={'succeeded':'GoToYellowMobileTorch', 'timeout':'problem'})
                        
# Go to Yellow Mobile Torch
            PreemptiveStateMachine.add('GoToYellowMobileTorch',
                      AmbiOmniDirectOrder2(Pose2D(0.600, 0.000, -pi/2)),
                      transitions={'succeeded':'ActionFireYellowMobileTorch', 'timeout':'problem'})
            
# Action Fire Yellow Mobile Torch     
            PreemptiveStateMachine.add('ActionFireYellowMobileTorch',
                      WaiterState(2.0),
                      transitions={'timeout':'PrepareRecalY'})










# Turn around Yellow Mobile Torch
            PreemptiveStateMachine.add('TurnEmptyYellowMobileTorch',
                      AmbiOmniDirectOrder2(Pose2D(0.600, 0.000, -5*pi/6)),
                     # AmbiOmniDirectOrder_cpoint(0.100, 0.000, 0.0, 
                      #                           0.600,-0.100,-5*pi/6),
                      transitions={'succeeded':'PushEmptyYellowMobileTorch', 'timeout':'problem'})
                        
# Push Empty Yellow Mobile Torch
            PreemptiveStateMachine.add('PushEmptyYellowMobileTorch',
                      AmbiOmniDirectOrder2(Pose2D(-0.200, -0.800, -5*pi/6)),
                      transitions={'succeeded':'EscapeEmptyYellowMobileTorch', 'timeout':'problem'})

# Escape Empty Yellow Mobile Torch 
            PreemptiveStateMachine.add('EscapeEmptyYellowMobileTorch',
                      AmbiOmniDirectOrder2(Pose2D(0.200, -0.800, -pi/2)),
                      transitions={'succeeded':'GoToYellowTorchBot', 'timeout':'problem'})

# Go to Yellow Torch Bot
# !!! Autom V1 used for this action, because of Autom V2 bug

            PreemptiveStateMachine.add('GoToYellowTorchBot',
                      AmbiOmniDirectOrder2(Pose2D(0.200, -1.000 + Robot2014.FRONT_SIDE.x, -pi/2)),
                      transitions={'succeeded':'ActionFireYellowTorchBot', 'timeout':'problem'})
 
# Action Fires Yellow Torch Bot     
            PreemptiveStateMachine.add('ActionFireYellowTorchBot',
                      WaiterState(1.5),
                      transitions={'timeout':'GoToYFB'})

# Escape Yellow Torch Bot

# Go to Yellow Fire Bot
            PreemptiveStateMachine.add('GoToYFB',
                      AmbiOmniDirectOrder2(Pose2D(0.600 - Robot2014.FRONT_SIDE.x,-0.600,0), vmax = 1.0),
                      transitions={'succeeded':'PickYFB', 'timeout':'problem'})

# Pick Yellow Fire Bot            
            PreemptiveStateMachine.add('PickYFB',
                      WaiterState(1.5),
                      transitions={'timeout':'PrepareRecalY'})









#Saut de strat pour tester les ordres de recalage.


            PreemptiveStateMachine.add('PrepareRecalY',
                      AmbiOmniDirectOrder2(Pose2D(0.800,0.500,pi/2)),
                      transitions={'succeeded':'RecalY', 'timeout':'problem'})
            
            PreemptiveStateMachine.add('RecalY',
                      AmbiRecalOnBorderYellow("FRUITBASKET",Data.color),
                      transitions={'recaled':'EscapeRecalY', 'non-recaled':'problem','problem':'problem'})  

            PreemptiveStateMachine.add('EscapeRecalY',
                      AmbiOmniDirectOrder2(Pose2D(1.200,0.400,pi/4)),
                      transitions={'succeeded':'PrepareRecalX', 'timeout':'problem'})
             
            PreemptiveStateMachine.add('PrepareRecalX',
                      AmbiOmniDirectOrder2(Pose2D(1.350,0.550,0), vmax=0.3),
                      transitions={'succeeded':'RecalX', 'timeout':'problem'})
            
            PreemptiveStateMachine.add('RecalX',
                      AmbiRecalOnBorderYellow("RIGHT",Data.color),
                      transitions={'recaled':'PrepareUnloadFireInStartArea', 'non-recaled':'problem','problem':'problem'})

            
# Enter Start Area
            PreemptiveStateMachine.add('PrepareUnloadFireInStartArea',
                      AmbiOmniDirectOrder2(Pose2D(1.225, 0.525, pi/3), vmax=0.3),
                      transitions={'succeeded':'DropFiresStartArea', 'timeout':'problem'})

# Drop Fires in stock in Start area       
            PreemptiveStateMachine.add('DropFiresStartArea',
                      WaiterState(2.5),
                      transitions={'timeout':'ExitStartArea'})

# Exit Start Area
            PreemptiveStateMachine.add('ExitStartArea',
                      AmbiOmniDirectOrder2(Pose2D(0.900, 0.000, pi/3)),
                      transitions={'succeeded':'Timer', 'timeout':'problem'})
            
# Timer             
            PreemptiveStateMachine.add('Timer',
                      WaiterState(15),
                      transitions={'timeout':'GoToYFM'})
                                                           
# Go to Yellow Fire Mid
            PreemptiveStateMachine.add('GoToYFM',
                      AmbiOmniDirectOrder2(Pose2D(1.100, 0.000 - Robot2014.FRONT_SIDE.x, -pi/2)),
                      transitions={'succeeded':'PickYFM', 'timeout':'problem'})

# Pick Yellow Fire Mid            
            PreemptiveStateMachine.add('PickYFM',
                      WaiterState(1.5),
                      transitions={'timeout':'GoToYellowTorchMid'})
            
# Go to Yellow Torch Mid
            PreemptiveStateMachine.add('GoToYellowTorchMid',
                      AmbiOmniDirectOrder2(Pose2D(1.500 - Robot2014.FRONT_SIDE.x, 0.200, 0)),
                      transitions={'succeeded':'ActionFireYellowTorchMid', 'timeout':'problem'})
 
# Action Fires Yellow Torch Mid     
            PreemptiveStateMachine.add('ActionFireYellowTorchMid',
                      WaiterState(1.5),
                      transitions={'timeout':'TreeAvoidance'})

# Tree avoidance
            PreemptiveStateMachine.add('TreeAvoidance',
                      AmbiOmniDirectOrder2Pass(Pose2D(1.200, -0.300, 0), vpasse=1.0),
                      transitions={'succeeded':'GoToHeatBot', 'timeout':'problem'})
            
# Go to Heat Bot
            PreemptiveStateMachine.add('GoToHeatBot',
                      AmbiOmniDirectOrder2(Pose2D(1.300 + Robot2014.RIGHT_SIDE.x, -0.750 - Robot2014.FRONT_SIDE.x, -pi/4)),
                      transitions={'succeeded':'DropFiresHeatBot', 'timeout':'problem'})
            
# Drop Fires in stock in Heat bot        
            PreemptiveStateMachine.add('DropFiresHeatBot',
                      WaiterState(5.0),
                      transitions={'timeout':'WaitBeforeNext'})
            
# Useless timer            
            PreemptiveStateMachine.add('WaitBeforeNext',
                      WaiterState(1.5),
                      transitions={'timeout':'endOpening'})