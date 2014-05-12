#!/usr/bin/env python
# -*- coding: latin-1 -*-

#libraries for ROS
import roslib; roslib.load_manifest('arp_master')

from arp_master import *
from arp_master.util import *
from arp_master.fsmFramework import *

#
# This is a standard state to publish a single message on a topic.
# @param topic       : the topic name in which the message has to be published (/Ubiquity/Machin/truc)
# @param msgType     : the type of the message to publish (Bool, UInt8, Float64, ...)
# @param msg         : your message content
class SendOnTopic(smach.State):
    def __init__(self,topic, msgType, msg):
        smach.State.__init__(self, outcomes=['done'])
        #remember the message to send it in the execute step
        self.msg = msg
        #remember topic name to print debug info
        self.topic = topic
        
        #get an handler on a topic publisher configured with constructor params
        self.topicPublisher = rospy.Publisher(topic,msgType)
    
    def execute(self, userdata):
        rospy.loginfo("Publishing on topic " + self.topic)
        self.topicPublisher.publish( self.msg)
        return 'done'
