#!/usr/bin/env python

#libraries for ROS
import roslib; roslib.load_manifest('arp_master')
from arp_master import *

from Table2014 import *
from Robot2014 import *

from arp_master.util import *
from arp_master.fsmFramework import *
from arp_master.commonStates import *

#from arp_master.commonStates.EndMatchPreempter import *
#from arp_master.commonStates.Waiting import *

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
                      AmbiOmniDirectOrder2(Pose2D(0.000, 1.000 - 0.214 - 0.050, pi+0.428)),
                      transitions={'succeeded':'StickFrescos', 'timeout':'problem'})
            
            self.setInitialState('GoToFresco')
            
            #Utiliser un ordre d'approche pour gerer la collision avec le mur (peut venir du timeout precedent)
                        
            # Stick Fresco
            PreemptiveStateMachine.add('StickFrescos',
                      AmbiOmniDirectOrder2(Pose2D(0.000, 1.000 - 0.214 + 0.200, pi+0.428),vmax=0.3),
                      transitions={'succeeded':'problem', 'timeout':'QuitFresco'})

            # Quit Fresco
            PreemptiveStateMachine.add('QuitFresco',
                      AmbiOmniDirectOrder2(self.getEntryYellowPose()),
                      transitions={'succeeded':'endFrescos', 'timeout':'problem'})
            