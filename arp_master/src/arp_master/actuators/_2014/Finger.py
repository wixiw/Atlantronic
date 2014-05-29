#!/usr/bin/env python
# -*- coding: latin-1 -*-

#libraries for ROS
import roslib; roslib.load_manifest('arp_master')

from arp_master.strat_2014.common_2014.Robot2014 import *

from arp_master.util import *
from arp_master.fsmFramework import *
from arp_master.commonStates import *
from arp_master.strat_2014 import *
from arp_master.actuators import *

 
#
# This state machine allows to put the cannon in its initial state.
#

class DefaultFingerState(smach.StateMachine):
    def __init__(self, p_side):
            smach.StateMachine.__init__(self, outcomes=['done','problem'])
            
            with self:      
                smach.StateMachine.add('FingerDefaultPosition',
                       AmbiFingerCommand(p_side, Robot2014.fingerLeftYellowPos['UP']),
                       transitions={'succeeded':'done', 'problem':'problem'})
 
            

#===============================================================================
# Use this state machine to pick an object with the "p_side" finger (in the yellow config) automatically symetrized for match color
#===============================================================================


class AmbiFingerPickObject(smach.StateMachine):
    def __init__(self, p_side):
            smach.StateMachine.__init__(outcomes=['picked','blocked','notFound'])
            
            with self:      
                smach.StateMachine.add('GoDown',
                       AmbiFingerCommand(p_side, Robot2014.fingerLeftYellowPos['DOWN']),
                       transitions={'succeeded':'SearchItemToPick', 'problem':'RetryGoDown'})

                # ==> Blocking Point D

                smach.StateMachine.add('SearchItemToPick',
                       AmbiWaitFingerOmronValue(p_side, True, 10.0),
                       transitions={'triggered':'SuckMyLovingPump', 'timeout':'GoUpAfterNotFound'})    

                # ==> Functional Error A

                smach.StateMachine.add('SuckMyLovingPump',
                       FingerPumpCommand(p_side, suctionPower['HOLD']),
                       transitions={'done':'WaitABit'})  

                smach.StateMachine.add('WaitABit',
                       WaiterState(0.3),
                       transitions={'timeout':'WaitForObjectPresent'})
                
                smach.StateMachine.add('WaitForObjectPresent',
                       FingerWaitForObjectPresent(p_side, p_timeout),
                       transitions={'object_present':'GoUp', 'timeout':'ReTrySuckObject'})                  

                # ==> Blocking Point B
                
                smach.StateMachine.add('GoUp',
                       AmbiFingerCommand(p_side, Robot2014.fingerLeftYellowPos['UP']),
                       transitions={'succeeded':'CheckPresence', 'problem':'GoDownbeforeRetryGoUp'})
                
                # ==> Blocking Point C
                
                smach.StateMachine.add('CheckPresence',
                       FingerWaitForObjectPresent(p_side, 0),
                       transitions={'triggered':'picked', 'timeout':'notFound'})

#Functional Error A: Raising the finger to return to initial state
                smach.StateMachine.add('GoUpAfterNotFound',
                       AmbiFingerCommand(p_side, Robot2014.fingerLeftYellowPos['UP']),
                       transitions={'succeeded':'notFound', 'problem':'blocked'})  

#Blocking Point B: If object is not detected by pump after sucking, probably because objet was not parralel to the ground
                smach.StateMachine.add('ReTrySuckObject',
                       FingerPumpCommand(p_side, suctionPower['HOLD']),
                       transitions={'done':'MoveUp'})  
                
                smach.StateMachine.add('MoveUp',
                       AmbiFingerCommand(p_side, Robot2014.fingerLeftYellowPos['UP']),
                       transitions={'succeeded':'MoveToFloor', 'problem':'blocked'})

                smach.StateMachine.add('MoveToFloor',
                       AmbiFingerCommand(p_side, Robot2014.fingerLeftYellowPos['FLOOR']),
                       transitions={'succeeded':'WaitAWhile', 'problem':'blocked'})

                smach.StateMachine.add('WaitAWhile',
                       WaiterState(0.3),
                       transitions={'timeout':'CheckObjectPresence'})
                
                smach.StateMachine.add('CheckObjectPresence',
                       FingerWaitForObjectPresent(p_side, p_timeout),
                       transitions={'object_present':'GoUp', 'timeout':'blocked'})                  
        
#Blocking Point C: If something is stucked between the robot and the finger, the finger can not go up => retry
#Entrypoint is Finger is blocked in rising state
                smach.StateMachine.add('GoDownbeforeRetryGoUp',
                       AmbiFingerCommand(p_side, Robot2014.fingerLeftYellowPos['DOWN']),
                       transitions={'succeeded':'WaitRetryGoUp', 'problem':'blocked'})
                
                smach.StateMachine.add('WaitRetryGoUp',
                       WaiterState(0.3),
                       transitions={'timeout':'RetryGoUp'})
                
                smach.StateMachine.add('RetryGoUp',
                       AmbiFingerCommand(p_side, Robot2014.fingerLeftYellowPos['UP']),
                       transitions={'succeeded':'CheckPresence', 'problem':'blocked'})
                
#Blocking Point D: If something is between the finger and the floor, the finger can not go down => retry
#Entrypoint is Finger is blocked in descending state

# Il faut reculer, abaisser sufisamment le doigt pour pousser un eventuel objet deplacable (torche, feu en biais, bouchon, balle)
# A faire dans strat PickingState
                smach.StateMachine.add('GoUpAfterRetryGoDown',
                       AmbiFingerCommand(p_side, Robot2014.fingerLeftYellowPos['UP']),
                       transitions={'succeeded':'WaitRetryGoDown', 'problem':'RetryGoDown'})
                
                smach.StateMachine.add('WaitRetryGoDown',
                       WaiterState(0.3),
                       transitions={'timeout':'ReleaseObject'})
                
                smach.StateMachine.add('RetryGoDown',
                       AmbiFingerCommand(p_side, Robot2014.fingerLeftYellowPos['DOWN']),
                       transitions={'succeeded':'ReleaseObject', 'problem':'blocked'})
                
                smach.StateMachine.add('ReleaseObject',
                       FingerPumpCommand(p_side, suctionPower['IDLE']),
                       transitions={'done':'SearchItemToPick'})   
 
#===============================================================================
# Use this state to unload an object hold with the "p_side" finger (in the yellow config) automatically symetrized for match color
#===============================================================================

class AmbiUnloadObjectOnThefFloor(smach.StateMachine):
    def __init__(self, p_side):
            smach.StateMachine.__init__(outcomes=['released','blocked','notFound'])
            
            with self:

                smach.StateMachine.add('CheckObjectPresence',
                       FingerWaitForObjectPresent(p_side, p_timeout),
                       transitions={'object_present':'GoDown', 'timeout':'notFound'}) 
                
                smach.StateMachine.add('GoDown',
                       AmbiFingerCommand(p_side, Robot2014.fingerLeftYellowPos['DOWN']),
                       transitions={'succeeded':'StopPumping', 'problem':'RetryGoDown'})

                smach.StateMachine.add('StopPumping',
                       FingerPumpCommand(p_side, suctionPower['IDLE']),
                       transitions={'done':'CheckPresence'})  
                
                smach.StateMachine.add('CheckPresence',
                       FingerWaitForObjectPresent(p_side, 0),
                       transitions={'triggered':'released', 'timeout':'notFound'})
                
#===============================================================================
# Use this state to wait for an object to be present in the suction cup
# @param String p_side : the side of the finger on the robot
#===============================================================================

class FingerWaitForObjectPresent(ReceiveFromTopic):
    def __init__(self, p_side, p_timeout):
        ReceiveFromTopic.__init__(self, "/Ubiquity/"+p_side+"FingerPump/object_present", Bool,
                                  outcomes=['object_present'], p_timeout=p_timeout) 
        
    def executeTransitions(self):
        #if no message has been received yet, just wait again
        if self.msg is None:
            return
        #end with success if last message is a True value
        if self.msg.data is True:
            return 'object_present'  
        #else continue to wait

#===============================================================================
# Use this state to drive the pump on the finger
# @param String p_side :      the side of the finger on the robot
# @param Integerp_power :     the suction power of the pump in [50;100] in percent
#===============================================================================

class FingerPumpCommand(StaticSendOnTopic):
    def __init__(self, p_side, p_power):
        StaticSendOnTopic.__init__(self, "/Ubiquity/"+p_side+"FingerPump/suction_power", UInt8, UInt8(p_power)) 

#===============================================================================
# This state allows to send configuration to the finger dynamixel
# 
# @param String p_side               : the side of the cannon (Left or Right)
# @param Percentage p_fingerTorque   : shooting finger torque in [20;100]
#===============================================================================

class FingerTorqueConfig(DynamixelTorqueConfig):
    def __init__(self, p_side, p_fingerTorque):
        DynamixelTorqueConfig.__init__(self, p_side + "Finger", p_fingerTorque)
        
          
#
# This state allows to send a blocking command to a front finger, the order is always given for the yellow configuration        
# @param Double p_position           :  finger command
#
class AmbiFingerCommand(AmbiDynamixelGoto):
    def __init__(self, p_side, p_position):
        AmbiDynamixelGoto.__init__(self,
                                   [ AmbiDynamixelCmd(p_side, "Finger",p_position) ]
                                    )          

#
# Use this state to drive the pump on the finger depending on the match color
# @param Integer p_power :     the suction power of the pump in [50;100] in percent
#
class AmbiFingerPumpCommand(DynamicSendOnTopic):
    def __init__(self, p_side, p_power):
        DynamicSendOnTopic.__init__(self) 
    
    #return the name of the omron to listen depending on the color and the side defined in the yellow config
    #@param String p_side : the side on the yellow
    #@param String p_color : match color
    def getName(self, p_side, p_color):
        return toAmbiSide(p_side, p_color) + "FingerPump"
    
    #Overrided to provide the topic name and the message from the AmbiDynamixelCmd
    def publish(self):
        topicPublisher = rospy.Publisher("/Ubiquity/"+self.getName(Data.color)+"/suction_power", Float32)
        topicPublisher.publish( Float32(self.cmd.getPositionCmd(color)) )#


# Use this state to wait an Omron on the finger to be in an expected state depending on the match color
class AmbiWaitFingerOmronValue(AmbiWaitForOmronValue):
    def __init__(self, p_side, p_awaitedValue, p_timeout):
        AmbiWaitForOmronValue.__init__(self, AmbiOmron(p_side, "FingerOmron"), p_awaitedValue, p_timeout) 

