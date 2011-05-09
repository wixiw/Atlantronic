#!/usr/bin/env python

import roslib; roslib.load_manifest('arp_master')
import rospy
import smach
import smach_ros
import smach_msgs



class CyclicState(smach.StateMachine):
    def __init__(self,outcomes):
        smach.StateMachine.__init__(self,outcomes)
        
    def execute(self,userdata):
        global stateMachineRate
        rospy.loginfo("executeIn")
        self.executeIn()
        while(not rospy.is_shutdown()):
            rospy.loginfo("executeWhile")
            self.executeWhile()
            rospy.loginfo("executeTransition")
            trans=self.executeTransitions()
            if trans!=None:
                rospy.loginfo("executeOut")
                self.executeOut()
                return trans
            rospy.loginfo("Sleep")
            #stateMachineRate.sleep()
        rospy.logerr("boucle d'etat cassee par le shutdown")
        
    
    def executeIn(self):
        return
    
    def executeWhile(self):
        return
    
    def executeTransitions(self):
        return
    
    def executeOut(self):
        return