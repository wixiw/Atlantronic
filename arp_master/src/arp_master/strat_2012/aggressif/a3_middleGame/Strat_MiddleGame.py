
#libraries for ROS
import roslib; roslib.load_manifest('arp_master')
from arp_master import *

class MiddleGame(PreemptiveStateMachine):
    def __init__(self):
        PreemptiveStateMachine.__init__(self,outcomes=['endMiddleGame'])
        with self:      
            PreemptiveStateMachine.addPreemptive('EndMatchPreemption',
                                             EndMatchPreempter(-Robot2012.END_GAME_DELAY),
                                             transitions={'endMatch':'endMiddleGame'})
    
            PreemptiveStateMachine.add('BackFromMiddleObjects',
                      BackFromHalfMiddleObjects(),
                      transitions={'end':'CloseFingersABit', 'problem':'endMiddleGame'})
            self.setInitialState('BackFromMiddleObjects')
             
            PreemptiveStateMachine.add('CloseFingersABit',
                      AmbiClawFingerOrder(0.25, Robot2012.FINGER_CLOSE,
                                     Robot2012.CLAW_CLOSE, Robot2012.CLAW_CLOSE), 
                      transitions={'succeeded':'ThrowUp', 'timeout':'ThrowUp'})
            
            PreemptiveStateMachine.add('ThrowUp',
                      AmbiOmniDirectOrder(1.100,0.150,pi/5),
                      transitions={'succeeded':'OpenFingers', 'timeout':'OpenFingers'})

            PreemptiveStateMachine.add('OpenFingers',
                      FingersOnlyState('open'), 
                      transitions={'succeeded':'OpenAll', 'timeout':'OpenAll'})
            
            PreemptiveStateMachine.add('OpenAll',
                      FingerClawState('open'), 
                      transitions={'succeeded':'Back', 'timeout':'Back'})
            
            PreemptiveStateMachine.add('Back',
                      AmbiOmniDirectOrder(0.950,-0.100,pi/2),
                      transitions={'succeeded':'CloseFingersAndClaws', 'timeout':'CloseFingersAndClaws'})
            
            PreemptiveStateMachine.add('CloseFingersAndClaws',
                      FingerClawState('close'),
                      transitions={'succeeded':'PrepareBot', 'timeout':'endMiddleGame'}) 
            
            PreemptiveStateMachine.add('PrepareBot',
                      AmbiOmniDirectOrder(0.55,-0.45,pi/2),
                      transitions={'succeeded':'AntiWorkBotTotem', 'timeout':'endMiddleGame'})


            PreemptiveStateMachine.add('AntiWorkBotTotem',
                      AntiWorkBotCloseTotem(),
                      transitions={'endTotem':'ThrowUpBotCloseTotem', 'problem':'endMiddleGame'}) 

            PreemptiveStateMachine.add('ThrowUpBotCloseTotem',
                      ThrowUpBotCloseTotem(),
                      transitions={'end':'PrepareTop', 'problem':'endMiddleGame'})

            PreemptiveStateMachine.add('PrepareTop',
                      AmbiOmniDirectOrder(0.55,0.55,pi/2),
                      transitions={'succeeded':'CleanTopCloseTotem', 'timeout':'endMiddleGame'})

            PreemptiveStateMachine.add('CleanTopCloseTotem',
                      CleanTopCloseTotem(),
                      transitions={'endClean':'AntiWorkTopTotem', 'problem':'endMiddleGame'})

            PreemptiveStateMachine.add('AntiWorkTopTotem',
                      AntiWorkTopCloseTotem(),
                      transitions={'endTotem':'ThrowUpTopCloseTotem', 'problem':'endMiddleGame'}) 

            PreemptiveStateMachine.add('ThrowUpTopCloseTotem',
                      ThrowUpTopCloseTotem(),
                      transitions={'end':'endMiddleGame', 'problem':'endMiddleGame'})

            
            #cas d'erreur
            PreemptiveStateMachine.add('Debloque',
                      DeblocReloc(),
                      transitions={'endDeblocReloc':'endMiddleGame'})
            ####### A MODIFIER POUR ASSURER NON ARRET
