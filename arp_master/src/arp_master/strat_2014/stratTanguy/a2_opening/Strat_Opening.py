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
                      AmbiOmniDirectOrder2(1.300, 0.400, 0, 1.0),
                      transitions={'succeeded':'GayCamping', 'timeout':'problem'})
    
            #self.setInitialState('EscapeStartArea')
            
            
            PreemptiveStateMachine.add('GotoZero',
                      AmbiOmniDirectOrder2(0.000, 0.000, 0, 0.0),
                      transitions={'succeeded':'WaitWLA', 'timeout':'problem'})
            self.setInitialState('GotoZero')
            PreemptiveStateMachine.add('WaitWLA',
                      WaiterState(5),
                      transitions={'timeout':'TurnWLA'})
            PreemptiveStateMachine.add('TurnWLA',
                      AmbiTurnOrder(-pi),
                      transitions={'succeeded':'WaitWLAInf', 'timeout':'problem'})
            PreemptiveStateMachine.add('WaitWLAInf',
                      WaiterState(50),
                      transitions={'timeout':'endOpening'})


# Go to Gay Camping point, align for DoubleShoot_1
            PreemptiveStateMachine.add('GayCamping',
                      AmbiOmniDirectOrder2(0.000, 0.400, -pi/2 + 0.08, 1.0),
                      transitions={'succeeded':'DoubleShoot_1', 'timeout':'problem'})
# DoubleShoot_1            
            PreemptiveStateMachine.add('DoubleShoot_1',
                      WaiterState(1.5),
                      transitions={'timeout':'TurnDoubleShoot_2'})
            
# 2x DoubleShoots on Mammoths, turn of 0.085 RAD = 5 DEG

            #  Turn DoubleShoot_2
            PreemptiveStateMachine.add('TurnDoubleShoot_2',
                      AmbiTurnOrder(-pi/2 + 0.00),
                      transitions={'succeeded':'DoubleShoot_2', 'timeout':'problem'})
            
            # DoubleShoot_2            
            PreemptiveStateMachine.add('DoubleShoot_2',
                      WaiterState(1.5),
                      transitions={'timeout':'TurnDoubleShoot_3'})
            
            #  Turn DoubleShoot_3
            PreemptiveStateMachine.add('TurnDoubleShoot_3',
                      AmbiTurnOrder(-pi/2 - 0.08),
                      transitions={'succeeded':'DoubleShoot_3', 'timeout':'problem'})
            
            # DoubleShoot_3           
            PreemptiveStateMachine.add('DoubleShoot_3',
                      WaiterState(1.5),
                      transitions={'timeout':'GoToFresco'})
                        
# Go to Fresco
            PreemptiveStateMachine.add('GoToFresco',
                      AmbiOmniDirectOrder2(0.120, 1.000 + Robot2014.REAR_SIDE.x, -pi/2, 1.0),
                      transitions={'succeeded':'StickFresco', 'timeout':'problem'})

# Stick Fresco (tempo 3s)
            PreemptiveStateMachine.add('StickFresco',
                      WaiterState(3.0),
                      transitions={'timeout':'GayCamping2'})
            
# Go to Gay Camping point
            PreemptiveStateMachine.add('GayCamping2',
                      AmbiOmniDirectOrder2(0.000, 0.400, -pi/2, 1.0),
                      transitions={'succeeded':'GoToYellowMobileTorch', 'timeout':'problem'})
                        
# Go to RFT, push RFT
#            PreemptiveStateMachine.add('RFT',
#                      AmbiOmniDirectOrder2(-0.600 - Robot2014.RIGHT_SIDE.x, 0.300, -pi/2, 1.0),
#                      transitions={'succeeded':'GoToYellowMobileTorch', 'timeout':'problem'})

# Go to Yellow Mobile Torch
            PreemptiveStateMachine.add('GoToYellowMobileTorch',
                      AmbiOmniDirectOrder2(0.600, 0.000, -pi/2, 1.0),
                      transitions={'succeeded':'ActionFireYellowMobileTorch', 'timeout':'problem'})
            
# Action Fire Yellow Mobile Torch     
            PreemptiveStateMachine.add('ActionFireYellowMobileTorch',
                      WaiterState(3.5),
                      transitions={'timeout':'GoToYellowTorchBot'})
            
# Go to Yellow Torch Bot
            PreemptiveStateMachine.add('GoToYellowTorchBot',
                      AmbiOmniDirectOrder2(0.200, -1.000 + Robot2014.FRONT_SIDE.x, -pi/2, 1.0),
                      transitions={'succeeded':'ActionFireYellowTorchBot', 'timeout':'problem'})
 
# Action Fires Yellow Torch Bot     
            PreemptiveStateMachine.add('ActionFireYellowTorchBot',
                      WaiterState(1.5),
                      transitions={'timeout':'GoToYFB'})
                        
# Go to Yellow Fire Bot
            PreemptiveStateMachine.add('GoToYFB',
                      AmbiOmniDirectOrder2(0.600, -0.600, 0, 1.0),
                      transitions={'succeeded':'PickYFB', 'timeout':'problem'})
# Pick Yellow Fire Bot            
            PreemptiveStateMachine.add('PickYFB',
                      WaiterState(1.5),
                      transitions={'timeout':'GoToYFM'})
            
# Go to Yellow Fire Mid
            PreemptiveStateMachine.add('GoToYFM',
                      AmbiOmniDirectOrder2(1.100, 0.000, -pi/2, 1.0),
                      transitions={'succeeded':'GoToYellowTorchMid', 'timeout':'problem'})
            
# Go to Yellow Torch Mid
            PreemptiveStateMachine.add('GoToYellowTorchMid',
                      AmbiOmniDirectOrder2(1.500 - Robot2014.FRONT_SIDE.x, 0.200, 0, 1.0),
                      transitions={'succeeded':'ActionFireYellowTorchMid', 'timeout':'problem'})
 
# Action Fires Yellow Torch Mid     
            PreemptiveStateMachine.add('ActionFireYellowTorchMid',
                      WaiterState(1.5),
                      transitions={'timeout':'WaitBeforeNext'})

# Useless timer            
            PreemptiveStateMachine.add('WaitBeforeNext',
                      WaiterState(1.5),
                      transitions={'timeout':'endOpening'})