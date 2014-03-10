#!/usr/bin/env python

#libraries for ROS
import roslib; roslib.load_manifest('arp_master')
from arp_master import *

from Table2014 import *
from Robot2014 import *
from arp_master.commonStates.EndMatchPreempter import *
from arp_master.commonStates.Waiting import *

#This action stick frescos, as a total of 2.
#In order to use this action you have to go to the entry point StickFrescosState.getEntryPoint()
class StickFrescosState(PreemptiveStateMachine):

    @staticmethod
    def getEntryYellowPose():
        return Pose2D(0.000, 0.400, -pi/2);
    
    def __init__(self):
        PreemptiveStateMachine.__init__(self,outcomes=['endFrescos','problem'])
        with self:      
            PreemptiveStateMachine.addPreemptive('EndMatchPreemption',
                                             EndMatchPreempter(-Robot2014.SWITCH_TO_EOG_DELAY),
                                             transitions={'endMatch':'endFrescos'})
            
            # Approach Fresco
            PreemptiveStateMachine.add('GoToFresco',
                      AmbiOmniDirectOrder2(Pose2D(0.120, 1.000 + Robot2014.REAR_SIDE.x, -pi/2),vmax=0.3),
                      transitions={'succeeded':'StickFrescos', 'timeout':'problem'})
            
            self.setInitialState('GoToFresco')
            
            #Utiliser un ordre d'approche pour gerer la collision avec le mur (peut venir du timeout precedent)
            
            
            # Stick Fresco (tempo 3s)
            PreemptiveStateMachine.add('StickFrescos',
                      WaiterState(3.0),
                      transitions={'timeout':'QuitFresco'})

            # Quit Fresco
            PreemptiveStateMachine.add('QuitFresco',
                      AmbiOmniDirectOrder2(self.getEntryYellowPose(),vmax=0.3),
                      transitions={'succeeded':'endFrescos', 'timeout':'problem'})
            