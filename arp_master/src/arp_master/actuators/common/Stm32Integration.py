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
    def __init__(self, pumpName, suctionPower):
        SendOnTopic.__init__(self, "/Ubiquity/"+pumpName+"/suction_power", UInt8, UInt8(suctionPower))

#
# You may drive a dynamixel with this state. It send a single command, it is *not* periodic.
# @param dynamixelName : the name of the dynamixel to drive :
#     _ LeftFinger
#     _ RightFinger
#     _ LeftCannonShooter
#     _ RightCannonShooter
#     _ LeftCannonStocker
#     _ RightCannonStocker
# @param position : position cmd in [-pi;pi] in radians
class DynamixelGotoPosition(SendOnTopic):
    def __init__(self, dynamixelName, position):
        SendOnTopic.__init__(self, "/Ubiquity/"+dynamixelName+"/position_cmd", Float32, Float32(position))