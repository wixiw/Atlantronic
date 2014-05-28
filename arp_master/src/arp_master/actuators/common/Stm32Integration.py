#!/usr/bin/env python
# -*- coding: latin-1 -*-

#libraries for ROS
import roslib; roslib.load_manifest('arp_master')

from arp_master import *
from arp_master.util import *
from arp_master.fsmFramework import *
from arp_master.commonStates import *
from std_msgs.msg import *


class ResetStm32(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['succeeded','failed'])
        self.setReset_srv=rospy.ServiceProxy("/Ubiquity/resetStm32",EmptyWithSuccess)

    def execute(self, userdata):
        try:
            self.setReset_srv();
            return 'succeeded';
        except rospy.ServiceException, e:
            rospy.logerr("ResetStm32 failed to reset e=")
            return 'failed'
    
#
# You may drive power on the stm32 with this command.
# @param Bool p_powerOn : set to true to requiere power
#
class SendStm32PowerCmd(StaticSendOnTopic):
    def __init__(self, p_powerOn):
        StaticSendOnTopic.__init__(self, "/Ubiquity/stm32_power_request", Bool, Bool(p_powerOn))

#
# You may inform the stm32 that the robot is ready to fight.
#
class SendReadyForMatch(StaticSendOnTopic):
    def __init__(self):
        StaticSendOnTopic.__init__(self, "/Ubiquity/ready_for_match", Bool, Bool(True))

#
# You may inform the stm32 that the robot is initialized
#
class InformInitialized(StaticSendOnTopic):
    def __init__(self):
        StaticSendOnTopic.__init__(self, "/Master/initialized", Bool, Bool(True))

#
# You may wait for color to be entered by the user
#
class WaitStm32ColorChoice(ReceiveFromTopic):
    def __init__(self):
        ReceiveFromTopic.__init__(self, "/Ubiquity/color", String, p_outcomes=['color_choice_done'], p_timeout=90)

    def executeTransitions(self):    
        #if no message has been received yet, just wait again
        if self.msg is None:
            return
            
        color = self.msg.color
 
        if ((color == "red") and (Inputs.getstart() == 1)):
            rospy.loginfo("User has chosen RED")
            return 'color_choice_done' 
        
        if ((color == "yellow") and (Inputs.getstart() == 1)):
            rospy.loginfo("User has chosen YELLOW")
            return 'color_choice_done' 
        
        #else : continue to wait


        
#
# You may wait for power to be effectively in a predefined state.
# @param Bool p_expectedPower : set to true wait power on or false to wait power off
#
class WaitStm32PowerCmd(ReceiveFromTopic):
    def __init__(self, p_expectedPower):
        ReceiveFromTopic.__init__(self, "/Ubiquity/powerState", PowerStatusMsg, p_outcomes=['power_state_reached'], p_timeout=5)
        #remember power
        self.expectedPower = p_expectedPower

    def executeTransitions(self):    
        #if no message has been received yet, just wait again
        if self.msg is None:
            return
        #end with success if the power is as expected
        if self.msg.isPowerOn is self.expectedPower:
            return 'power_state_reached' 
        #if the AU is pushed : warn the user
        if self.msg.isEmergencyStopActive is True:
            rospy.logerror("AU pushed !!!!!")
            return
        #else : continue to wait

#
# You may drive a pump with this state. It send a single command, it is *not* periodic.
# @param String pumpName         : the name of the pump to drive
# @param Integer suctionPower    : drive the suction power in percent in the range [50;100]
#
class SendPumpCmd(StaticSendOnTopic):
    def __init__(self, p_pumpName, p_suctionPower):
        StaticSendOnTopic.__init__(self, "/Ubiquity/"+p_pumpName+"/suction_power", UInt8, UInt8(p_suctionPower))

#
# You may drive a dynamixel with this state. It send a single command, it is *not* periodic.
# Basically it means that you are *NOT* at the desired position when you exit this state
# @param dynamixelName : String - the name of the dynamixel to drive
# @param position : Double - position cmd in [-pi;pi] in radians
#
class DynamixelNonBlockingPositionCmd(StaticSendOnTopic):
    def __init__(self, p_dynamixelName, p_position):
        StaticSendOnTopic.__init__(self, "/Ubiquity/"+p_dynamixelName+"/position_cmd", Float32, Float32(p_position))

#
# You may configure a dynamixel torque with this state. It send a single command, it is *not* periodic.
# Basically it means that the torque cmd is just sent but not executed when you exit the state
# @param String - dynamixelName     : the name of the dynamixel to configure
# @param Percentage - torque        : the torque in % in [20;100]
#
class DynamixelTorqueConfig(StaticSendOnTopic):
    def __init__(self, p_dynamixelName, p_torque):
        StaticSendOnTopic.__init__(self, "/Ubiquity/"+p_dynamixelName+"/max_torque", UInt8, UInt8(p_torque))

#
# You may drive a dynamixel with this state. It send a single command, it is *not* periodic.
# Basically it means that you are *NOT* at the desired position when you exit this state
# The dynamixel has 2s to reach its position
# @param dynamixelName : String - the name of the dynamixel to drive
#
class WaitDynamixelReachedPosition(ReceiveFromTopic):
    def __init__(self, p_dynamixelName):
        ReceiveFromTopic.__init__(self, "/Ubiquity/"+p_dynamixelName+"/state", DynamixelState,
            p_outcomes=['pos_reached','stucked'], p_timeout=2.0)

    def executeTransitions(self):    
        #if no message has been received yet, just wait again
        if self.msg is None:
            print("!!!!!!! *** "+self.topicName+" WaitDynamixelReachedPosition : None")
            return
        #end with success if the dynamixel is target_reached
        if self.msg.target_reached is True:
            print("!!!!!!! *** "+self.topicName+" WaitDynamixelReachedPosition : Position Reached")
            return 'pos_reached' 
        #if the servo is stucked end with failure
        if self.msg.stucked is True:
            print("!!!!!!! *** "+self.topicName+" WaitDynamixelReachedPosition : Stucked")
            return 'stucked'
        else : #continue to wait
            print("!!!!!!! *** "+self.topicName+" WaitDynamixelReachedPosition : Continue to wait")
         
#    
# Use this state to send a blocking "goto" command to list of dynamixels
# The dynamixels has 2s (cumulative in the number of dynamixels) to reach their positions
# @param dynamixelName : List(String) - the name of the dynamixels to drive
# @param position : List(Double) - position cmd in [-pi;pi] in radians
#   
class DynamixelGoto(smach.StateMachine):
    def __init__(self,p_dynamixelNameList, p_positionList):
        smach.StateMachine.__init__(self,outcomes=['succeeded','problem'])
        with self: 
            for i,dynamixelName in enumerate(p_dynamixelNameList):
                stateName = 'SendCommandTo' + dynamixelName
                if i >= len(p_dynamixelNameList)-1:
                    transitionName = 'Wait'
                else:
                    transitionName = 'SendCommandTo' + p_dynamixelNameList[i+1]
                smach.StateMachine.add(stateName,
                    DynamixelNonBlockingPositionCmd(dynamixelName, p_positionList[i]),
                    transitions={ 'done' : transitionName })
        
            smach.StateMachine.add('Wait',
                    WaiterState(0.1),
                    transitions={'timeout':'Wait' + p_dynamixelNameList[0] + 'PositionReached'})
              
            for j,dynamixelName in enumerate(p_dynamixelNameList):   
                stateName = 'Wait' + dynamixelName + 'PositionReached'
                if i >= len(p_dynamixelNameList)-1:
                    transitionName = 'succeeded'
                else:
                    transitionName = 'Wait' + p_dynamixelNameList[j+1] + 'PositionReached'
                    
                smach.StateMachine.add(stateName,
                    WaitDynamixelReachedPosition(dynamixelName),
                    transitions={'pos_reached':transitionName, 'stucked':'problem', 'timeout':'problem'})

#
# Use this state to wait an Omron to be in an expected state
#
# @param String p_omronName         : the omron to listen
# @param Boolean p_awaitedValue     : 'True' or 'False' The value you are waiting until the timeout.
# @param Double p_timeout           : the timeout to limit infinite wait
#
class WaitForOmronValue(ReceiveFromTopic):
    def __init__(self, p_omronName, p_awaitedValue, p_timeout):
        ReceiveFromTopic.__init__(self, "/Ubiquity/"+p_omronName+"/object_present", Bool,
                                  outcomes=['triggered'], timeout=p_timeout) 

        #remember reference value
        self.awaitedValue = p_awaitedValue
        
    def executeTransitions(self):
        #if no message has been received yet, just wait again
        if self.msg is None:
            return
        #end with success if last message is the awaited value, else continue to wait
        if self.msg.data == self.awaitedValue:
            return 'triggered'  

#
#This is a utility class for the AmbiDynamixelGoto class
# 
# @param String p_side             : the side of the dynamixel in the yellow configuration
# @param String p_name             : the name of the dynamixel (without the Left/Right prefix)
# @param Double p_position         : the position of the dynamixel on the left side in the yellow configuration
#
class AmbiDynamixelCmd():
    def __init__(self, p_side, p_name, p_position):
        self.side = p_side
        self.name = p_name
        self.position = p_position
        
    #return the name of the dynamixel to drive depending on the color
    #@param String p_color : match color
    def getName(self, p_color):
        #rospy.loginfo("AmbiDynamixelCmd color : " + p_color)
        #rospy.loginfo("AmbiDynamixelCmd side : " + self.side)
        #rospy.loginfo("AmbiDynamixelCmd name : " + self.name)
        return toAmbiSide(self.side, p_color) + self.name
        
    def getYellowName(self):
        return self.side + "Yellow" + self.name;
        
    #return the position cmd of the dynamixel to drive depending on the color
    #@param String p_color : match color
    def getPositionCmd(self, p_color):
        ambiSide = toAmbiSide(self.side, p_color)
        if ambiSide is "Left":
            return self.position
        if ambiSide is "Right":
            return -self.position
                
#
# You may drive a dynamixel with this state depending on color and side. 
# It send a single command, it is *not* periodic.
# Basically it means that you are *NOT* at the desired position when you exit this state
#
# @param AmbiDynamixelCmd p_ambiDynamixelCmd   : the name of the dynamixel to drive (defined on the left side in the yellow color)
#
class AmbiDynamixelNonBlockingPositionCmd(DynamicSendOnTopic):
    def __init__(self, p_ambiDynamixelCmd):
        DynamicSendOnTopic.__init__(self)
        
        #remember command
        self.cmd = p_ambiDynamixelCmd
        
    #Overrided to provide the topic name and the message from the AmbiDynamixelCmd
    def publish(self):
        #TODO a corriger
        color = "yellow"
        print("AmbiDynamixelNonBlockingPositionCmd publishing in : /Ubiquity/"+self.cmd.getName(color)+"/position_cmd")
        #rospy.loginfo("AmbiDynamixelNonBlockingPositionCmd value : " + str(self.cmd.getPositionCmd(color)))
        topicPublisher = rospy.Publisher("/Ubiquity/"+self.cmd.getName(color)+"/position_cmd", Float32)
        topicPublisher.publish( Float32(self.cmd.getPositionCmd(color)) )
        

#
# You may drive a dynamixel with this state. It send a single command, it is *not* periodic.
# Basically it means that you are *NOT* at the desired position when you exit this state
# The dynamixel has 2s to reach its position
#
# @param dynamixelName : String - the name of the dynamixel to drive
#
class AmbiWaitDynamixelReachedPosition(WaitDynamixelReachedPosition):
    def __init__(self, p_ambiDynamixelCmd):
        WaitDynamixelReachedPosition.__init__(self, "notDefinedYet")
        
        #remember command
        self.cmd = p_ambiDynamixelCmd

    def executeIn(self):
        #TODO a corriger
        color = "yellow"
        self.topicName = "/Ubiquity/"+self.cmd.getName(color)+"/state"
        ReceiveFromTopic.executeIn(self)
        return

        

#    
# Use this state to send a blocking "goto" command to list of dynamixels on the left in the yellow configuration
# The dynamixels has 2s (cumulative in the number of dynamixels) to reach their positions
#
# @param List(AmbiDynamixelCmd) p_ambiDynamixelCmdList      : the list of commands to do, see AmbiDynamixelCmd
#   
class AmbiDynamixelGoto(smach.StateMachine):
    def __init__(self, p_ambiDynamixelCmdList):
        smach.StateMachine.__init__(self,outcomes=['succeeded','problem'])
        with self: 
            for i,ambiDynamixelCmd in enumerate(p_ambiDynamixelCmdList):
                
                stateName = 'AmbiSendCommandTo' + ambiDynamixelCmd.getYellowName()
                if i >= len(p_ambiDynamixelCmdList)-1:
                    transitionName = 'Wait'
                else:
                    transitionName = 'AmbiSendCommandTo' + p_ambiDynamixelCmdList[i+1].getYellowName()
                #print("[i="+str(i)+"]####################  AmbiDynamixelGoto stateName="+stateName + " transition="+transitionName)
                smach.StateMachine.add(stateName,
                    AmbiDynamixelNonBlockingPositionCmd(ambiDynamixelCmd),
                    transitions={ 'done' : transitionName })
        
            smach.StateMachine.add('Wait',
                    WaiterState(0.2),
                    transitions={'timeout':'AmbiWait' + p_ambiDynamixelCmdList[0].getYellowName() + 'PositionReached'})
              
            for j,ambiDynamixelCmd in enumerate(p_ambiDynamixelCmdList):   
                stateName = 'AmbiWait' + ambiDynamixelCmd.getYellowName() + 'PositionReached'
                if j >= len(p_ambiDynamixelCmdList)-1:
                    transitionName = 'succeeded'
                else:
                    transitionName = 'AmbiWait' + p_ambiDynamixelCmdList[j+1].getYellowName() + 'PositionReached'
                #print("[j="+str(j)+"]####################  AmbiDynamixelGoto stateName="+stateName + " transition="+transitionName)
                smach.StateMachine.add(stateName,
                    AmbiWaitDynamixelReachedPosition(ambiDynamixelCmd),
                    transitions={'pos_reached':transitionName, 'stucked':'problem', 'timeout':'problem'})
                
#
# This class represents an Omron efined by its side in a symetric pair depending on the match color
class AmbiOmron():
    def __init__(self, p_side, p_omronName):
        self.side = p_side
        self.name = p_omronName
        
    #return the name of the omron depending on the color
    #@param String p_color : match color
    def getName(self, p_color):
        return toAmbiSide(self.side, p_color) + self.name
        

#
# Use this state to wait an Omron on the left side in the yellow configuration to be in an expected state. 
#
# @param AmbiOmron p_ambiOmron      : the omron described in the Ambi formalism
# @param Boolean p_awaitedValue     : 'True' or 'False' The value you are waiting until the timeout.
# @param Double p_timeout           : the timeout to limit infinite wait
#
class AmbiWaitForOmronValue(WaitForOmronValue):
    def __init__(self, p_ambiOmron, p_awaitedValue, p_timeout):
        WaitForOmronValue.__init__(self, "NoName", p_awaitedValue, p_timeout) 
        
        #remember ambi omron
        self.ambiOmron = p_ambiOmron
        
    def executeIn(self):
        #TODO a corriger
        color = "yellow"
        self.topicName = "/Ubiquity/"+self.ambiOmron.getName(color)+"/object_present"
        WaitForOmronValue.executeIn(self)
        return
 