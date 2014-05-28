#!/usr/bin/env python
# -*- coding: latin-1 -*-

#libraries for ROS
import roslib; roslib.load_manifest('arp_master')

from arp_master.strat_2014.common_2014.Robot2014 import *

from arp_master.util import *
from arp_master.fsmFramework import *
from arp_master.commonStates import *
from arp_master.strat_2014 import *
from arp_master.actuators._2014 import *


#This state machine is configuring all torques on dynamixels
class ActuatorConfigStateMachine(smach.StateMachine):
    def __init__(self):
        smach.StateMachine.__init__(self,outcomes=['done'])

        
        with self:      
            #Move all dynamixels
            for index, dynamixelName in enumerate(Robot2014.dynamixelList):
                stateName = 'Configure' + dynamixelName
                if index >= len(Robot2014.dynamixelList)-1:
                    transitionName = 'done'
                else:
                    transitionName = 'Configure' + Robot2014.dynamixelList[index+1]
                    
                smach.StateMachine.add(stateName,
                          DynamixelTorqueConfig(dynamixelName, Robot2014.dynamixelMaxTorqueList[dynamixelName]),
                          transitions={'done': transitionName})
                
                        

                
class PrepareActuators(smach.StateMachine):
    def __init__(self):
        smach.StateMachine.__init__(self,outcomes=['prepared','problem'])
        
        with self:
            smach.StateMachine.add('ActuatorsConfig',
                      ActuatorConfigStateMachine(),
                      transitions={'done':'ActuatorsSelfTest'})

            smach.StateMachine.add('ActuatorsSelfTest',
                      SelfTest(),
                      transitions={'done':'prepared'})
            

                               