#!/usr/bin/env python
# -*- coding: latin-1 -*-

#libraries for ROS
import roslib; roslib.load_manifest('arp_master')
from arp_master import *

from arp_master.util import *
from arp_master.fsmFramework import *
from arp_master.commonStates import *
from arp_master.strat_2014.common_2014 import *



class Recal2014_Y_Fruitbasket(LocalStrategicAction):
    @staticmethod
    def getEntryYellowPoseStatic():
        return Pose2D(0.700,0.400,pi/2);
    
    def getEntryYellowPose(self):
        return self.getEntryYellowPoseStatic();
    
    def __init__(self):
        LocalStrategicAction.__init__(self, Robot2014.SWITCH_TO_EOG_DELAY)
        with self:          
            # Recal
            LocalStrategicAction.add('Recal',
                    AmbiRecalOnBorderYellow("FRUITBASKET"),
                    transitions={'recaled':'ReturnToEntryPoint', 
                                 'non-recaled':'ReturnToEntryPoint',
                                 'problem':'ReturnToEntryPoint'})  
            self.setInitialState('Recal')
            
            #Return to entry point
            LocalStrategicAction.add('ReturnToEntryPoint',
                    AmbiOmniDirectOrder2(self.getEntryYellowPoseStatic(), vmax=Robot2014.motionSpeeds['Average']),
                    transitions={'succeeded':'succeeded', 'timeout':'EmergencyReturn'})
            
            #Failure case : on a quelqu'un au cul : translater sur le cote
            LocalStrategicAction.add('EmergencyReturn',
                    AmbiOmniDirectOrder2(Pose2D(1.200,0.400,pi/2), vmax=Robot2014.motionSpeeds['Carefull']),
                    transitions={'succeeded':'succeeded', 'timeout':'Retry1'})

            #re-failure, on retry une fois le point de sortie, sinon on abandonne.
            LocalStrategicAction.add('Retry1',
                    AmbiOmniDirectOrder2(self.getEntryYellowPoseStatic(), vmax=Robot2014.motionSpeeds['Average']),
                    transitions={'succeeded':'succeeded', 'timeout':'failed'})
            
            
            #TODO j'en suis la : verifier le point d'entree
class Recal2014_X_SelfTorchMid(LocalStrategicAction):
    @staticmethod
    def getEntryYellowPoseStatic():
        return Pose2D(1.100, 0.200, 0);
    
    def getEntryYellowPose(self):
        return self.getEntryYellowPoseStatic();
    
    def __init__(self):
        LocalStrategicAction.__init__(self, Robot2014.SWITCH_TO_EOG_DELAY)
        with self:          
            # Recal
            LocalStrategicAction.add('Recal',
                    AmbiRecalOnBorderYellow("RIGHT"),
                    transitions={'recaled':'ReturnToEntryPoint', 
                                 'non-recaled':'ReturnToEntryPoint',
                                 'problem':'ReturnToEntryPoint'})  
            self.setInitialState('Recal')
            
            #Return to entry point
            LocalStrategicAction.add('ReturnToEntryPoint',
                    AmbiOmniDirectOrder2(self.getEntryYellowPoseStatic(), vmax=Robot2014.motionSpeeds['Average']),
                    transitions={'succeeded':'succeeded', 'timeout':'EmergencyReturn'})
            
            #Failure case : on a quelqu'un au cul : translater sur le cote
            LocalStrategicAction.add('EmergencyReturn',
                    AmbiOmniDirectOrder2(Pose2D(1.300,0.500,0.0), vmax=Robot2014.motionSpeeds['Carefull']),
                    transitions={'succeeded':'succeeded', 'timeout':'Retry1'})    
            
            #re-failure, on retry une fois le point de sortie, sinon on abandonne.
            LocalStrategicAction.add('Retry1',
                    AmbiOmniDirectOrder2(self.getEntryYellowPoseStatic(), vmax=Robot2014.motionSpeeds['Average']),
                    transitions={'succeeded':'succeeded', 'timeout':'failed'})
            
