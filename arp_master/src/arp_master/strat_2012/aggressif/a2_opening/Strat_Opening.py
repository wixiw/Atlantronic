#!/usr/bin/env python

#libraries for ROS
import roslib; roslib.load_manifest('arp_master')

from arp_master import *


class CacaVerdun(PreemptiveStateMachine):
    def __init__(self):
        PreemptiveStateMachine.__init__(self, outcomes=['retryFarBottle', 'endStraf','problem'])
        with self:
            PreemptiveStateMachine.addPreemptive('EndMatchPreemption',
                                             EndMatchPreempter(-Robot2012.END_GAME_DELAY),
                                             transitions={'endMatch':'problem'})
            
            PreemptiveStateMachine.add('ChooseState',
                      ChooseState(),
                      transitions={'blockedIncloseBottle':'GoAwayFromBottle', 
                                   'bockedAfterMiddle':'Turn',
                                   'timeout':'problem'})
            self.setInitialState('ChooseState')
            
            PreemptiveStateMachine.add('GoAwayFromBottle',
                      AmbiOpenLoopOrder(0,-0.200,0,
                               1),
                      transitions={'succeeded':'retryFarBottle', 'timeout':'retryFarBottle'})
            
            PreemptiveStateMachine.add('Turn',
                  AmbiTurnOrder(pi/2-pi/12),
                  transitions={'succeeded':'GetCenterGoldBar', 'timeout':'problem'})
            
            PreemptiveStateMachine.add('GetCenterGoldBar',
                      AmbiOmniDirectOrder(Table2012.P_GOLDBAR_CENTER.x, Table2012.P_GOLDBAR_CENTER.y,pi/2),
                      transitions={'succeeded':'CloseFingers', 'timeout':'problem'})
            
            PreemptiveStateMachine.add('CloseFingers',
                      FingersOnlyState('close'), 
                      transitions={'succeeded':'Straf', 'timeout':'Straf'})
            
            PreemptiveStateMachine.add('Straf',
                      AmbiOmniDirectOrder(0.750, -0.450,pi/5),
                      transitions={'succeeded':'endStraf', 'timeout':'problem'})
            
            
class ChooseState(CyclicState):
    def __init__(self):
        CyclicState.__init__(self, outcomes=['blockedIncloseBottle','bockedAfterMiddle'])
        self.RED_MIDDLE_PASSED = 0.0

    def executeTransitions(self):
        if Data.color == "red":
            if Inputs.getx() <= self.RED_MIDDLE_PASSED:
                return "bockedAfterMiddle"
            else:
                return "blockedIncloseBottle"
        else:
            if Inputs.getx() >= self.RED_MIDDLE_PASSED:
                return "bockedAfterMiddle"
            else:
                return "blockedIncloseBottle"

#######################################################################################




class Opening(PreemptiveStateMachine):
    def __init__(self):
        PreemptiveStateMachine.__init__(self, outcomes=['endOpening', 'problem'])
        with self:
            PreemptiveStateMachine.addPreemptive('EndMatchPreemption',
                                             EndMatchPreempter(-Robot2012.END_GAME_DELAY),
                                             transitions={'endMatch':'problem'})
            
            PreemptiveStateMachine.add('EscapeStart',
                      AmbiOmniDirectOrder(0.875, 0.700,-pi),
                      transitions={'succeeded':'BourrineCloseBottle', 'timeout':'BourrineCloseBottle'})
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
                  transitions={'succeeded':'CloseFingers', 'timeout':'CacaVerdun'})
########  
            PreemptiveStateMachine.add('CacaVerdun',
                  CacaVerdun(),
                  transitions={'retryFarBottle':'GoFarBottle2', 
                               'endStraf':'endOpening',
                               'problem':'problem'})
 
            PreemptiveStateMachine.add('GoFarBottle2',
                  AmbiOmniDirectOrder(Table2012.P_BOTTLE_FAR.x+ 0.040,-0.700,-pi),
                  transitions={'succeeded':'CloseFingers', 'timeout':'problem'})
 
 
 ########################""
 
            PreemptiveStateMachine.add('CloseFingers',
                      FingersOnlyState('close'), 
                      transitions={'succeeded':'FarBottle', 'timeout':'FarBottle'})
            
            PreemptiveStateMachine.add('FarBottle',
                  FarAntiBottleState(),
                  transitions={'endBottle':'Turn', 'problem':'problem'})
            
            PreemptiveStateMachine.add('Turn',
                  AmbiTurnOrder(pi/5),
                  transitions={'succeeded':'BackFromMiddleObjects', 'timeout':'problem'})
           
            PreemptiveStateMachine.add('BackFromMiddleObjects',
                      BackFromHalfMiddleObjects(),
                      transitions={'end':'endOpening', 'problem':'problem'})
           
           
           
            #cas d'erreur
            PreemptiveStateMachine.add('Debloque',
                      DeblocReloc(),
                      transitions={'endDeblocReloc':'problem'})
            
 

            