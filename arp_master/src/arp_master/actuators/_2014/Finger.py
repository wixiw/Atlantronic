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

#TODO
##
##
## Remettre le preemptive
## En faire un vrai action server, là c'est un peu compliqué d'interomptre une action

class FingerMainStateMachine(PreemptiveStateMachine):
    def __init__(self, p_side):
        PreemptiveStateMachine.__init__(self,outcomes=['end','problem'])

        #Configuration of reference positions
        self.refPos = { 'UP' : pi/4.0,
                        'DOWN' : 0.0}
        #Configuration of reference suction powers
        self.refSuction = { 'HOLD':100,
                           'IDLE':0 }
        
        with self:      
            PreemptiveStateMachine.add('EmptyUpPos',
                      FingerAutoSideCommand(p_side, self.refPos['UP']),
                      transitions={'succeeded':'WaitCommand', 'problem':'problem'})
            
            PreemptiveStateMachine.add('WaitCommand',
                      FingerWaitForStratRequest(p_side),
                      transitions={'pick':'SearchItemToPick',
                                   'unload':'UnloadItem'})
                        
            #prise
            PreemptiveStateMachine.add('SearchItemToPick',
                      FingerAutoSideCommand(p_side, self.refPos['DOWN']),
                      transitions={'succeeded':'WaitDown', 'problem':'problem'})    
            
            PreemptiveStateMachine.add('SearchItemToPick',
                      FingerPumpCommand(p_side, self.refSuction['HOLD']),
                      transitions={'done':'WaitItemPresent'})   
            
            PreemptiveStateMachine.add('WaitItemPresent',
                      FingerWaitForObjectPresent(),
                      transitions={'object_present':'UpLoaded'})  
            
            PreemptiveStateMachine.add('UpLoaded',
                      FingerAutoSideCommand(p_side, self.refPos['UP']),
                      transitions={'succeeded':'WaitCommand', 'problem':'problem'})  
            
            
            #depose
            PreemptiveStateMachine.add('UnloadItem',
                      FingerAutoSideCommand(p_side, self.refPos['DOWN']),
                      transitions={'succeeded':'ReleaseItem', 'problem':'problem'})    
            
            PreemptiveStateMachine.add('ReleaseItem',
                      FingerPumpCommand(p_side, self.refSuction['IDLE']),
                      transitions={'done':'WaitABit'})    
            
            PreemptiveStateMachine.add('WaitABit',
                      WaiterState(0.5),
                      transitions={'timeout':'EmptyUpPos'})    
            
#Remplacer UserDebugTrigger par vraie etat une fois fini            
#
##Test?
#
#            PreemptiveStateMachine.add('Test',
#                      UserDebugTrigger("Presence feu ?"),
#                      transitions={'continue':'FullDownDrop'})
#            
#            PreemptiveStateMachine.add('FullDownDrop',
#                      UserDebugTrigger("Position basse pleine depose"),
#                      transitions={'continue':'EmptyDownDrop'})
#            
#            PreemptiveStateMachine.add('EmptyDownDrop',
#                      UserDebugTrigger("Position basse vide post depose"),
#                      transitions={'continue':'EmptyUpPos'})  
            
#
# Use this state to wait for a strat command
# Available commands are :
# _ pick
# _ unload
# @param String p_side : the side of the finger on the robot
#
class FingerWaitForStratRequest(ReceiveFromTopic):
    def __init__(self, p_side):
        ReceiveFromTopic.__init__(self, "/Ubiquity/"+p_side+"Finger/strat_command", String,
                                  outcomes=['pick','unload']) 
        
    def executeTransitions(self):
        #if no message has been received yet, just wait again
        if self.msg is None:
            return
        #end with success if last message is a known command
        if self.msg.data is 'pick':
            return 'pick'  
        if self.msg.data is 'unload':
            return 'unload' 

            
#
# Use this state to wait for an object to be present in the suction cup
# @param String p_side : the side of the finger on the robot
#
class FingerWaitForObjectPresent(ReceiveFromTopic):
    def __init__(self, p_side):
        ReceiveFromTopic.__init__(self, "/Ubiquity/"+p_side+"FingerPump/object_present", Bool,
                                  outcomes=['object_present']) 
        
    def executeTransitions(self):
        #if no message has been received yet, just wait again
        if self.msg is None:
            return
        #end with success if last message is a True value
        if self.msg.data is True:
            return 'object_present'  
        #else continue to wait


#
# Use this state to drive the pump on the finger
# @param String p_side :      the side of the finger on the robot
# @param Integerp_power :     the suction power of the pump in [50;100] in percent
#
class FingerPumpCommand(SendOnTopic):
    def __init__(self, p_side, p_power):
        SendOnTopic.__init__(self, "/Ubiquity/"+p_side+"FingerPump/suction_power", UInt8, UInt8(p_power)) 

          
# This state allows to send a blocking command to a front finger, the order is always given for a left side
# the other side is deduced           
# @param String p_side               : the side of the finger (Left or Right)
# @param Double p_position           :  finger command
class FingerAutoSideCommand(DynamixelGoto):
    def __init__(self, p_side, p_position):
        dynamixelList = [p_side + "Finger"]
        if p_side is 'Left':
            posList = [p_position]
        else:
            posList = [-p_position]
        DynamixelGoto.__init__(self, dynamixelList, posList)          

#
# This state allows to send configuration to the finger dynamixel
#
# @param String p_side               : the side of the cannon (Left or Right)
# @param Percentage p_fingerTorque   : shooting finger torque in [20;100]
#
class FingerTorqueConfig(DynamixelTorqueConfig):
    def __init__(self, p_side, p_fingerTorque):
        DynamixelTorqueConfig.__init__(self, p_side + "Finger", p_fingerTorque)
