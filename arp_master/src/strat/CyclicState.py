#!/usr/bin/env python

import roslib; roslib.load_manifest('arp_master')
import rospy
import smach
import smach_ros
import smach_msgs



class CyclicState(smach.StateMachine):
    def __init__(self):
        smach.StateMachine.__init__(self)
        
    def execute(self):
        global stateMachineRate
        executeIn()
        while(1):
            executeWhile()
            trans=executeTransitions()
            if trans!=None:
                return trans
            stateMachineRate.sleep()
    
    def executeIn(self):
        return
    
    def executeWhile(self):
        return
    
    def executeTransitions(self):
        return