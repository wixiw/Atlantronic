#!/usr/bin/env python
# -*- coding: latin-1 -*-

#libraries for ROS
import roslib; roslib.load_manifest('arp_master')

from arp_master import *
from arp_master.util import *
from arp_master.fsmFramework import *
from arp_master.commonStates import *
from std_msgs.msg import *

#
# You may drive a pump with this state. It send a single command, it is *not* periodic.
# @param pumpName : the name of the pump to drive :
#     _ LeftFingerPump
#     _ RightFingerPump
#     _ ArmPump
# @param suctionPower : drive the suction power in percent in the range [50;100]
class SendPumpCmd(SendOnTopic):
    def __init__(self, p_pumpName, p_suctionPower):
        SendOnTopic.__init__(self, "/Ubiquity/"+p_pumpName+"/suction_power", UInt8, UInt8(p_suctionPower))

#
# You may drive a dynamixel with this state. It send a single command, it is *not* periodic.
# Basically it means that you are *NOT* at the desired position when you exit this state
# @param dynamixelName : the name of the dynamixel to drive :
#     _ LeftFinger
#     _ RightFinger
#     _ LeftCannonShooter
#     _ RightCannonShooter
#     _ LeftCannonStocker
#     _ RightCannonStocker
# @param position : position cmd in [-pi;pi] in radians
class DynamixelNonBlockingPositionCmd(SendOnTopic):
    def __init__(self, p_dynamixelName, p_position):
        SendOnTopic.__init__(self, "/Ubiquity/"+p_dynamixelName+"/position_cmd", Float32, Float32(p_position))
      
#
# Use this state to wait an Omron to be in an expected state
# @param p_omronName : the omron to listen
# @param p_awaitedValue : 'True' or 'False' The value you are waiting until the timeout.
class WaitForOmronValue(CyclicState):
    def __init__(self, p_omronName, p_awaitedValue, p_timeout):
        CyclicState.__init__(self, outcomes=['triggered'], timeout=p_timeout) 
        #remember topic
        self.topicName = "/Ubiquity/"+p_omronName+"/object_present"
        
        #remember reference value
        self.awaitedValue = p_awaitedValue
        
        #Keep an handle on the subscribe, else it would be destructed
        self.subscriber = None
        
        #Buffer updated by the callback to be used when testing transitions
        self.objectPresent = False
    
    def callback(self,msg):
        self.objectPresent = msg.data
        
    #The topic is only listened when entering the state to prevent useless trigger when not active       
    def executeIn(self):
        self.subscriber = rospy.Subscriber(self.topicName, Bool, self.callback)
        
    #Leaving the handle will destruct the subscriber
    def executeOut(self):
        self.subscriber.unregister()
        self.objectPresent = False
        self.subscriber = None
        
    def executeTransitions(self):
       if self.objectPresent == self.awaitedValue:
            return 'triggered'  