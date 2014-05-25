#!/usr/bin/env python
# -*- coding: latin-1 -*-

#libraries for ROS
import roslib; roslib.load_manifest('arp_master')

from arp_master import *
from arp_master.util import *
from arp_master.fsmFramework import *

#
# This is a standard state to publish a single message on a topic.
# @param p_topic       : String - the topic name in which the message has to be published (/Ubiquity/Machin/truc)
# @param p_msgType     : the type of the message to publish (Bool, UInt8, Float64, ...)
# @param p_msg         : your message content
class SendOnTopic(smach.State):
    def __init__(self,p_topic, p_msgType, p_msg):
        smach.State.__init__(self, outcomes=['done'])
        #remember the message to send it in the execute step
        self.msg = p_msg
        #remember p_topic name to print debug info
        self.topic = p_topic
        
        #get an handler on a p_topic publisher configured with constructor params
        self.topicPublisher = rospy.Publisher(p_topic,p_msgType)
    
    def execute(self, userdata):
        rospy.loginfo("Publishing on topic " + self.topic)
        self.topicPublisher.publish( self.msg)
        return 'done'

#
# This is a standard state to register a listener on a topic. The registration is done in the entry action, 
# the unregistration is done in the exit action. Thus the callback is only called when the state is active.
#
# This class HAS TO be heritated with your specific stuff in the executeTransitions. The receive data is 
# in the self.msg member.
#
# @param p_topic       : String - the topic name in which the message has to be published (/Ubiquity/Machin/truc)
# @param p_msgType     : the type of the message to publish (Bool, UInt8, Float64, ...)
# @param p_outcomes  : (optional) List - list outcomes from the child state
# @param p_timeout   : (optional) Double - define the timeout after which the state is exited
#
class ReceiveFromTopic(CyclicState):
    def __init__(self,p_topic, p_msgType,
                 p_outcomes=[''],p_timeout=5):
        CyclicState.__init__(self, outcomes=p_outcomes, timeout=p_timeout) 
        
        #remember topic info
        self.topicName = p_topic
        self.topicMsgType = p_msgType
        
        #Keep an handle on the subscribe, else it would be destructed
        self.subscriber = None
        
        #Last received message
        self.msg = None
        
    #Called by ROS on message reception in the topic    
    def callback(self, p_msg):
        self.msg = p_msg
        
     #The topic is only listened when entering the state to prevent useless trigger when not active       
    def executeIn(self):
        self.subscriber = rospy.Subscriber(self.topicName, self.topicMsgType, self.callback)
        
    #stop listening to the stop to prevent useless callback activation
    def executeOut(self):
        self.subscriber.unregister()
        self.subscriber = None
