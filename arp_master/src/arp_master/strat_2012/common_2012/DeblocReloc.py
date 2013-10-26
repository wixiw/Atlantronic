#!/usr/bin/env python

#libraries for ROS
import roslib; roslib.load_manifest('arp_master')
from arp_master import *

from DynamixelActionState import *
from DeblocReloc import *
from Table2012 import *
from Robot2012 import *
from math import *

class DeblocReloc(PreemptiveStateMachine):
    def __init__(self):
        PreemptiveStateMachine.__init__(self,outcomes=['endDeblocReloc'])
        with self:      
            PreemptiveStateMachine.addPreemptive('EndMatchPreemption',
                                             EndMatchPreempter(Robot2012.END_GAME_DELAY),
                                             transitions={'endMatch':'endDeblocReloc'})
            
            #Rewind + rangement du doigt
            PreemptiveStateMachine.add('Debloque',
                      Rewind(1.0),
                      transitions={'succeeded':'UnSetVmaxRecover', 'timeout':'UnSetVmaxRecover'})
            
            self.setInitialState('Debloque')

            
            PreemptiveStateMachine.add('UnSetVmaxRecover', 
                      SetVMaxState(0.0),
                      transitions={'succeeded':'Close','timeout':'Close'}) 
            
            PreemptiveStateMachine.add('Close',
                      FingerClawState('close'), 
                      transitions={'succeeded':'ChooseToReloc', 'timeout':'ChooseToReloc'})  
            
            PreemptiveStateMachine.add('ChooseToReloc',
                    ChooseToReloc(),
                    transitions={'reloc':'TurnToBeacons', 'notReloc':'endDeblocReloc','timeout':'endDeblocReloc'})
            
            PreemptiveStateMachine.add('TurnToBeacons',
                    TurnToBeacons() , 
                    transitions={'succeeded':'WaitForReloc','timeout':'WaitForReloc'})
                                       
            PreemptiveStateMachine.add('WaitForReloc',
                    WaiterState(0.5),
                    transitions={'timeout':'IsRelocOk'})
                                                                                            
            PreemptiveStateMachine.add('IsRelocOk',
                    IsRelocOk(),
                    transitions={'fusion':'endDeblocReloc','odo':'endDeblocReloc','timeout':'endDeblocReloc'})                                                 
                                       
            

#l'etat qui regarde si on est localise ou pas
class ChooseToReloc(CyclicState):
    def __init__(self):
        CyclicState.__init__(self, outcomes=['reloc','notReloc','timeout'])
    

    def executeTransitions(self):
        if True: ############## A MODIFIER
            return 'reloc'
        else:
            return 'notReloc'

class IsRelocOk(CyclicState):
    def __init__(self):
        CyclicState.__init__(self, outcomes=['fusion','odo','timeout'])
    

    def executeTransitions(self):
        if True: ############## A MODIFIER
            return 'fusion'
        else:
            return 'odo'

    
class TurnToBeacons(CyclicActionState):  
    def __init__(self):
        CyclicActionState.__init__(self)
    def createAction(self):
        BeaconUp=AmbiPoseRed(Table2012.P_RED_BEACON_UP.x,Table2012.P_RED_BEACON_UP.y,0,Data.color)
        BeaconDown=AmbiPoseRed(Table2012.P_RED_BEACON_DOWN.x,Table2012.P_RED_BEACON_DOWN.y,0,Data.color)
        hUp=atan2(BeaconUp.y-Inputs.gety(),BeaconUp.x-Inputs.getx())
        hDown=atan2(BeaconDown.y-Inputs.gety(),BeaconDown.x-Inputs.getx())
        h=normalizeAngle(averageAngle(hUp,hDown)+pi) #le hokuyo regarde vers l'arriere
        
        self.cap( h )        