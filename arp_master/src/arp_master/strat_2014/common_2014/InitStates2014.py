#!/usr/bin/env python

#libraries for ROS
import roslib; roslib.load_manifest('arp_master')


from arp_master import *

from Table2014 import *

from arp_master.util import *
from arp_master.fsmFramework import *
from arp_master.commonStates import *
from arp_master.actuators import *

#from arp_master.commonStates.Strat_Initialisation import *
#from arp_master.commonStates.Strat_RecalOnBorder import *
#from arp_master.commonStates.SetPosition import *

####################################################################################################################
            
class StartSequence2014(smach.StateMachine):
    def __init__(self,startPosition):
        smach.StateMachine.__init__(self,outcomes=['gogogo','problem'])
        
        with self:
            smach.StateMachine.add('PrepareActuators',
                      PrepareActuators(),
                      transitions={'prepared':'SetInitialPosition'})

            smach.StateMachine.add('SetInitialPosition',
                      SetPositionState(Pose2D(1.350,0.500,0)),
                      transitions={'succeeded':'WaitForStartUnPlug', 'timeout':'problem'})
            
            smach.StateMachine.add('WaitForStartUnPlug',
                      WaitForStartUnplug(),
                      transitions={'startunplug':'RecalX', 'timeout':'problem'})

            smach.StateMachine.add('RecalX',
                      AmbiRecalOnBorderYellow("RIGHT",Data.color),
                      transitions={'recaled':'PrepareRecalY', 'non-recaled':'problem','problem':'problem'})
            
            smach.StateMachine.add('PrepareRecalY',
                      AmbiOmniDirectOrder2(Pose2D(0.900,0.500,pi/2), vmax = 0.3),
                      transitions={'succeeded':'RecalY', 'timeout':'problem'})
            
            smach.StateMachine.add('RecalY',
                      AmbiRecalOnBorderYellow("FRUITBASKET",Data.color),
                      transitions={'recaled':'PrepareGoHome', 'non-recaled':'problem','problem':'problem'})   
            
            smach.StateMachine.add('PrepareGoHome',
                      AmbiOmniDirectOrder2(Pose2D(1.100,0.350,-3*pi/4), vmax = 0.3),
                      transitions={'succeeded':'GoHome', 'timeout':'GoHome'})
            
            smach.StateMachine.add('GoHome',
                      AmbiOmniDirectOrder2(startPosition, vmax = 0.3),
                      transitions={'succeeded':'WaitAbit', 'timeout':'WaitAbit'})
            
            smach.StateMachine.add('WaitAbit',
                      WaiterState(1.0),
                      transitions={'timeout':'You'})
            
            smach.StateMachine.add('You',
                      AmbiOmniDirectOrder2( Table2014.P_YOU_HOU, vmax = 0.3),
                      transitions={'succeeded':'Hou', 'timeout':'Hou'})
                        
            smach.StateMachine.add('Hou',
                      AmbiOmniDirectOrder2(startPosition, vmax = 0.3),
                      transitions={'succeeded':'WaitForStart', 'timeout':'WaitForStart'})
            
            smach.StateMachine.add('WaitForStart', 
                       WaitForStart(),
                       transitions={'start':'WaitForMatch','timeout':'WaitForStart'})
            
            smach.StateMachine.add('WaitForMatch', 
                      WaitForMatch(),
                      transitions={'start':'gogogo', 'timeout':'WaitForMatch'})


####################################################################################################################
