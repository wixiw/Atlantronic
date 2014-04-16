#!/usr/bin/env python


#libraries for ROS
import roslib; roslib.load_manifest('arp_master')


from arp_master import *
import os
from SetPosition import *

from arp_master.fsmFramework import *


#
# This is the default a1 level state for any strategy. Except for testing purpose it should be overided in any strategy
#
##################################################


class AmbiRecalOnBorderYellow(smach.StateMachine):
    def __init__(self,borderName,color):
        
        recallWalls={'RIGHT':(1.500-RobotVierge.FRONT_SIDE.x,"FREE",0),
                 'LEFT':(-1.500+RobotVierge.FRONT_SIDE.x,"FREE",-pi),
                 'UP':("FREE",1.000-RobotVierge.FRONT_SIDE.x,pi/2),
                 'DOWN':("FREE",-1.000+RobotVierge.FRONT_SIDE.x,-pi/2),
                 'FRUITBASKET':("FREE",0.700-RobotVierge.FRONT_SIDE.x,pi/2)
                 }
        
        smach.StateMachine.__init__(self,outcomes=['recaled','non-recaled','problem'])
        
        if color=='yellow':
            pass
        elif color=='red':
            if borderName=="LEFT":
                borderName="RIGHT"
            if borderName=="RIGHT":
                borderName="LEFT"            
        else:
            rospy.logerr("AmbiRecalOnBorderYellow : default case : color not defined !!")
        
        with self:
            smach.StateMachine.add('ForwardOrder',
                                   ForwardOrder(dist=0.200,vmax=0.2),
                                   transitions={'succeeded':'setPosition', 'timeout':'setPosition'}) 
            #TODO le succeded devrait etre un probleme ! on doit toujours bloquer. mais si je fas ca la simul va merder
            smach.StateMachine.add('setPosition',
                                   SetPositionState(*recallWalls[borderName]),
                                   transitions={'succeeded':'BackwardOrder', 'timeout':'problem'})
            smach.StateMachine.add('BackwardOrder',
                                   BackwardOrder(dist=0.100,vmax=0.3),
                                   transitions={'succeeded':'recaled', 'timeout':'problem'})
            
       

