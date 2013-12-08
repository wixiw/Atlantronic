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
                      AmbiOmniDirectOrder2Pass( Pose2D(1.100, 0.400, pi) , 1.0 ),
                      transitions={'succeeded':'GoToYFT', 'timeout':'problem'})

            self.setInitialState('EscapeStartArea')
            
# Go to Yellow Fire Top
            PreemptiveStateMachine.add('GoToYFT',
                      AmbiOmniDirectOrder2Pass( Pose2D(0.600 + Robot2014.FRONT_SIDE.x, 0.400, pi), vpasse=1.0),
                      transitions={'succeeded':'PickYFT', 'timeout':'problem'})

# Pick Yellow Fire Top            
            PreemptiveStateMachine.add('PickYFT',
                      WaiterState(0.1),
                      transitions={'timeout':'GayCamping'})


# Go to Gay Camping point, align for ShootStates
            PreemptiveStateMachine.add('GayCamping',
                      AmbiOmniDirectOrder2(DoubleTargetShootState.getEntryYellowPose()),
                      transitions={'succeeded':'ShootBalls', 'timeout':'problem'})

#Shoot Balls
            PreemptiveStateMachine.add('ShootBalls',
                      DoubleTargetShootState(),
                      transitions={'endShoot':'StickFrescos', 'problem':'problem'})

#Go to stick to frescos entry point no requiered as it is the same as DoubleTargetShootState     
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
                      WaiterState(10.0),
                      transitions={'timeout':'TurnEmptyYellowMobileTorch'})

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
                      transitions={'timeout':'GoToStartArea'})

# Go to Start Area
            PreemptiveStateMachine.add('GoToStartArea',
                      AmbiOmniDirectOrder2Pass(Pose2D(0.900, 0.400, pi/3), vpasse=1.0),
                      transitions={'succeeded':'EnterStartArea', 'timeout':'problem'})
            
# Enter Start Area
            PreemptiveStateMachine.add('EnterStartArea',
                      AmbiOmniDirectOrder2(Pose2D(1.300 + Robot2014.RIGHT_SIDE.x, 0.550, pi/3)),
                      transitions={'succeeded':'DropFiresStartArea', 'timeout':'problem'})

# Drop Fires in stock in Start area       
            PreemptiveStateMachine.add('DropFiresStartArea',
                      WaiterState(2.5),
                      transitions={'timeout':'CleanZone'})
#
# WARNING BUG
#

# Clean zone before relocate Robot2014.LEFT_SIDE.x
            PreemptiveStateMachine.add('CleanZone',
                      AmbiOmniDirectOrder2(Pose2D(0.600, 0.700 - 0.212, 0)),
                      transitions={'succeeded':'Relocation', 'timeout':'problem'})

# Relocation
            PreemptiveStateMachine.add('Relocation',
                      AmbiOmniDirectOrder2(Pose2D(0.700, 0.700 + Robot2014.FRONT_SIDE.x, pi/2)),
                      transitions={'succeeded':'TimerRelocation', 'timeout':'problem'})

# Timer Relocation            
            PreemptiveStateMachine.add('TimerRelocation',
                      WaiterState(1.5),
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
                      AmbiOmniDirectOrder2Pass(Pose2D(1.200, -0.300, 0),vpasse=1.0),
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