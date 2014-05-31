#!/usr/bin/env python
# -*- coding: latin-1 -*-

#libraries for ROS
import roslib; roslib.load_manifest('arp_master')

from arp_master.strat_2014.common_2014.Robot2014 import *

from arp_master.fsmFramework import *
from arp_master.commonStates import *
from arp_master.strat_2014 import *
from arp_master.actuators import *                
    

class AmbiShootFirstBall(smach.StateMachine):
    def __init__(self, p_side, p_nbBalls):
            smach.StateMachine.__init__(self, outcomes=['shot','blocked'])    
        
            with self:  
                
                smach.StateMachine.add('StartShooter',
                       AmbiFingerSpeedCmd(p_side, Robot2014.cannonFingerSpeed['SHOOT']),
                       transitions={'done':'WaitFirstBallOut'})
                              
                smach.StateMachine.add('WaitFirstBallOut',
                           WaiterState(2.0),
                           transitions={'timeout':'StopShooter'})
                                     
                #Arret du cannon en success
                smach.StateMachine.add('StopShooter',
                       AmbiFingerSpeedCmd(p_side, Robot2014.cannonFingerSpeed['STOPPED']),
                       transitions={'done':'shot'})
                  
                #Arret du cannon en blocage et suppression du couple
                smach.StateMachine.add('EmergencyStopShooter',
                       AmbiFingerSpeedCmd(p_side, Robot2014.cannonFingerSpeed['STOPPED']),
                       transitions={'done':'EmergencyStopStocker'})  
                
                smach.StateMachine.add('EmergencyStopStocker',
                       CannonTorqueConfig(p_side, 0),
                       transitions={'done':'blocked'}) 


class AmbiShootNBalls(smach.StateMachine):
    def __init__(self, p_side, p_nbBalls):
            smach.StateMachine.__init__(self, outcomes=['shot','blocked'])    
        
            with self:  
                
                smach.StateMachine.add('StartShooter',
                       AmbiFingerSpeedCmd(p_side, Robot2014.cannonFingerSpeed['SHOOT']),
                       transitions={'done':'GetFirstBallFromStock'})
                              
                              
                #On commence la machine par les derniers etat car les transition entre 1=>2 et 2=>3 dependent du nombre d'etat
                              
                if p_nbBalls > 2:
                    #Chargement de la troisieme balle
                    smach.StateMachine.add('GetThirdBallFromStock',
                           AmbiGetNextBallFromStocker(p_side),
                           transitions={'succeeded':'PutThirdBallInCannon', 'blocked':'EmergencyStopShooter'})
                                    
                    smach.StateMachine.add('PutThirdBallInCannon',
                           AmbiPutNextBallInCannon(p_side),
                           transitions={'succeeded':'WaitThirdBallOut', 'blocked':'EmergencyStopShooter'})  
                    
                    smach.StateMachine.add('WaitThirdBallOut',
                           WaiterState(1.0),
                           transitions={'timeout':'StopShooter'})
                    
                    secondBallTransition = "GetThirdBallFromStock"
                else:
                    secondBallTransition = "StopShooter"
                    
                    
                if p_nbBalls > 1:
                    #Chargement de la seconde balle
                    smach.StateMachine.add('GetSecondBallFromStock',
                           AmbiGetNextBallFromStocker(p_side),
                           transitions={'succeeded':'PutSecondBallInCannon', 'blocked':'EmergencyStopShooter'})
                                    
                    smach.StateMachine.add('PutSecondBallInCannon',
                           AmbiPutNextBallInCannon(p_side),
                           transitions={'succeeded':'WaitSecondBallOut', 'blocked':'EmergencyStopShooter'})  
                    
                    smach.StateMachine.add('WaitSecondBallOut',
                           WaiterState(1.0),
                           transitions={'timeout':secondBallTransition})
                    
                    firstBallTransition = "GetSecondBallFromStock"
                else:
                    firstBallTransition = "StopShooter"    
                          
                          
                          
                smach.StateMachine.add('GetFirstBallFromStock',
                           AmbiGetNextBallFromStocker(p_side),
                           transitions={'succeeded':'PutFirstBallInCannon', 'blocked':'EmergencyStopShooter'})
                                     
                smach.StateMachine.add('PutFirstBallInCannon',
                       AmbiPutNextBallInCannon(p_side),
                       transitions={'succeeded':'WaitFirstBallOut', 'blocked':'EmergencyStopShooter'})
                
                smach.StateMachine.add('WaitFirstBallOut',
                           WaiterState(1.5),
                           transitions={'timeout':firstBallTransition})
                
                #Arret du cannon en success
                smach.StateMachine.add('StopShooter',
                       AmbiFingerSpeedCmd(p_side, Robot2014.cannonFingerSpeed['STOPPED']),
                       transitions={'done':'shot'})
                  
                #Arret du cannon en blocage et suppression du couple
                smach.StateMachine.add('EmergencyStopShooter',
                       AmbiFingerSpeedCmd(p_side, Robot2014.cannonFingerSpeed['STOPPED']),
                       transitions={'done':'EmergencyStopStocker'})  
                
                smach.StateMachine.add('EmergencyStopStocker',
                       CannonTorqueConfig(p_side, 0),
                       transitions={'done':'blocked'}) 


class AmbiPutNextBallInCannon(smach.StateMachine):
    def __init__(self, p_side):
            smach.StateMachine.__init__(self, outcomes=['succeeded','blocked'])    
        
            with self:  
                smach.StateMachine.add('PutBallInCannon',
                       AmbiCannonStockerCmd(p_side, Robot2014.cannonStockerLeftYellowPos['UNLOADING']),
                       transitions={'succeeded':'WaitBallInCannon', 'problem':'ReLoadBall'})
                #==> Blocking Point A
                                
                smach.StateMachine.add('WaitBallInCannon',
                       WaiterState(0.),
                       transitions={'timeout':'succeeded'}) 
                 
                #Blocking point A : when the ball don't fall from Stocker to Shooter
                smach.StateMachine.add('ReLoadBall',
                       AmbiCannonStockerCmd(p_side, Robot2014.cannonStockerLeftYellowPos['LOADING']),
                       transitions={'succeeded':'ReadyToShootNoFailure', 'problem':'blocked'})
                
                smach.StateMachine.add('ReadyToShootNoFailure',
                       AmbiCannonStockerCmd(p_side, Robot2014.cannonStockerLeftYellowPos['UNLOADING']),
                       transitions={'succeeded':'succeeded', 'problem':'blocked'})  


class AmbiGetNextBallFromStocker(smach.StateMachine):
    def __init__(self, p_side):
            smach.StateMachine.__init__(self, outcomes=['succeeded','blocked'])    
        
            with self:  
                smach.StateMachine.add('PutBallInCannon',
                       AmbiCannonStockerCmd(p_side, Robot2014.cannonStockerLeftYellowPos['LOADING']),
                       transitions={'succeeded':'WaitBallInStocker', 'problem':'Retry'})
                #==> Blocking Point A
                                
                smach.StateMachine.add('WaitBallInStocker',
                       WaiterState(0.),
                       transitions={'timeout':'succeeded'})
                
                #Blocking point A : when next ball prevent finger from moving
                smach.StateMachine.add('Retry',
                       AmbiCannonStockerCmd(p_side, Robot2014.cannonStockerLeftYellowPos['UNLOADING']),
                       transitions={'succeeded':'PutBallSecondTry', 'problem':'blocked'})
                
                smach.StateMachine.add('PutBallSecondTry',
                       AmbiCannonStockerCmd(p_side, Robot2014.cannonStockerLeftYellowPos['LOADING']),
                       transitions={'succeeded':'succeeded', 'problem':'blocked'})    
    
              
class CannonTorqueConfig(DynamixelTorqueConfig):
    def __init__(self, p_side, p_stockerTorque):
        DynamixelTorqueConfig.__init__(self,p_side + "CannonStocker", p_stockerTorque)

class AmbiCannonStockerCmd(AmbiDynamixelGoto):
    def __init__(self, p_side, p_stockerPosition):
        AmbiDynamixelGoto.__init__(self, [AmbiDynamixelCmd(p_side, "CannonStocker", p_stockerPosition)])
        
        
class FingerSpeedCmd(DynamixelNonBlockingSpeedCmd):
    def __init__(self, p_side, p_speed):
        DynamixelNonBlockingSpeedCmd.__init__(self, p_side + "CannonFinger" , p_speed)
        
        
class AmbiFingerSpeedCmd(AmbiDynamixelNonBlockingSpeedCmd):
    def __init__(self, p_side, p_speed):
        AmbiDynamixelNonBlockingSpeedCmd.__init__(self,AmbiDynamixelSpeedCmd(p_side, "CannonFinger", p_speed))
