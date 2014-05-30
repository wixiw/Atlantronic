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
                      transitions={'succeeded':'GoToYFT', 'timeout':'RetryEscapeStartArea'})
            self.setInitialState('EscapeStartArea')
            
            PreemptiveStateMachine.add('RetryEscapeStartArea',
                      AmbiOmniDirectOrder2Pass(Pose2D(Table2014.P_YOU_HOU.x, Table2014.P_YOU_HOU.y -0.050, Table2014.P_YOU_HOU.theta) ,vpasse=-1),
                      transitions={'succeeded':'GoToYFT', 'timeout':'nearlyEndMatch'})
            
                
# Go to Yellow Fire Top
            PreemptiveStateMachine.add('GoToYFT',
                      AmbiOmniDirectOrder2Pass( Pose2D(0.650 + Robot2014.FRONT_SIDE.x, 0.300, -5*pi/6),vpasse=-1),
                      transitions={'succeeded':'GotoGayCampingPoint', 'timeout':'nearlyEndMatch'})

# Go to Gay Camping point
            PreemptiveStateMachine.add('GotoGayCampingPoint',
                      AmbiOmniDirectOrder2(AmbiShootMammoth.getEntryYellowPoseStatic('Left', p_opponent_side = True)),
                      transitions={'succeeded':'PrepareFrescos', 'timeout':'nearlyEndMatch'})   

# Go to Frescos entry point
            PreemptiveStateMachine.add('PrepareFrescos',
                      AmbiOmniDirectOrder2(StickFrescosState.getEntryYellowPoseStatic()),
                      transitions={'succeeded':'StickFrescos', 'timeout':'nearlyEndMatch'})   
            
#Stick Frescos
            PreemptiveStateMachine.add('StickFrescos',
                      StickFrescosState(),
                      transitions={'succeeded':'ReturnInStartArea', 'failed':'nearlyEndMatch', 'almostEndGame':'nearlyEndMatch' })
#DEBUG WILLY POUR HOMOLOGATION EVITEMENT
            PreemptiveStateMachine.add('ReturnInStartArea',
                      AmbiOmniDirectOrder2(Table2014.P_YOU_HOU),
                      transitions={'succeeded':'CyclePoint', 'timeout':'CyclePoint'})
            
            #DEBUG WILLY POUR HOMOLOGATION EVITEMENT
            PreemptiveStateMachine.add('CyclePoint',
                      AmbiOmniDirectOrder2(Pose2D(0.600, -0.600, 0)),
                      transitions={'succeeded':'ReturnInStartArea', 'timeout':'ReturnInStartArea'})

####################################################################################################################
## End of Rush
####################################################################################################################
