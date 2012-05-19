#!/usr/bin/env python

#libraries for ROS
import roslib; roslib.load_manifest('arp_master')

from arp_master import *

class Opening(PreemptiveStateMachine):
    def __init__(self):
        PreemptiveStateMachine.__init__(self, outcomes=['endOpening', 'problem'])
        with self:
            PreemptiveStateMachine.addPreemptive('EndMatchPreemption',
                                             EndMatchPreempter(-Robot2012.END_GAME_DELAY),
                                             transitions={'endMatch':'problem'})
            
            PreemptiveStateMachine.add('EscapeStart',
                      AmbiOmniDirectOrder(0.875, 0.700,-pi),
                      transitions={'succeeded':'BourrineCloseBottle', 'timeout':'Debloque'})
            self.setInitialState('EscapeStart')
            
            PreemptiveStateMachine.add('BourrineCloseBottle',
                      AmbiOmniDirectOrder(Table2012.P_BOTTLE_CLOSE.x+0.058, - 1.050,-pi),
                      transitions={'succeeded':'OpenFingers', 'timeout':'OpenFingers'})
            self.setInitialState('EscapeStart')
            
            PreemptiveStateMachine.add('OpenFingers',
                      FingersOnlyState('open'), 
                      transitions={'succeeded':'GoFarBottle', 'timeout':'GoFarBottle'})
            
            PreemptiveStateMachine.add('GoFarBottle',
                  AmbiOmniDirectOrder(Table2012.P_BOTTLE_FAR.x+ 0.040,-0.700,-pi),
                  transitions={'succeeded':'CloseFingers', 'timeout':'endOpening'})
            
 
            PreemptiveStateMachine.add('CloseFingers',
                      FingersOnlyState('close'), 
                      transitions={'succeeded':'FarBottle', 'timeout':'FarBottle'})
            
            PreemptiveStateMachine.add('FarBottle',
                  FarAntiBottleState(),
                  transitions={'endBottle':'Turn', 'problem':'problem'})
            
            PreemptiveStateMachine.add('Turn',
                  AmbiTurnOrder(pi/5),
                  transitions={'succeeded':'endOpening', 'timeout':'problem'})
           
            #cas d'erreur
            PreemptiveStateMachine.add('Debloque',
                      DeblocReloc(),
                      transitions={'endDeblocReloc':'problem'})
            
 
