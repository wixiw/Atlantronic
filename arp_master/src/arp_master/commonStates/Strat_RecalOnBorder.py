#!/usr/bin/env python


#libraries for ROS
import roslib; roslib.load_manifest('arp_master')


from arp_master import *
import os
from SetPosition import *

from arp_master.fsmFramework import *
from MotorManagement import *

#
# This is the default a1 level state for any strategy. Except for testing purpose it should be overided in any strategy
#
##################################################


class AmbiRecalOnBorderYellow(smach.StateMachine):
    def __init__(self,borderName,color):
        
        recallWalls={'RIGHT':Pose2D(1.500-RobotVierge.FRONT_SIDE.x,"FREE",0),
                 'LEFT':Pose2D(-1.500+RobotVierge.FRONT_SIDE.x,"FREE",-pi),
                 'UP':Pose2D("FREE",1.000-RobotVierge.FRONT_SIDE.x,pi/2),
                 'DOWN':Pose2D("FREE",-1.000+RobotVierge.FRONT_SIDE.x,-pi/2),
                 'FRUITBASKET':Pose2D("FREE",0.700-RobotVierge.FRONT_SIDE.x,pi/2)
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
            smach.StateMachine.add('AlignTurrets',
                                   OpenLoopOrder(0.1,0.0,0.0, 
                                                 duration=0.1),
                                   transitions={'succeeded':'ForwardOrder', 'timeout':'problem'})

            smach.StateMachine.add('ForwardOrder',
                                   ForwardOrder(dist=0.500,vmax=0.2),
                                   transitions={'succeeded':'non-recaled', 'timeout':'setPosition'}) 
            #TODO le succeded devrait etre un probleme ! on doit toujours bloquer. mais si je fas ca la simul va merder
            smach.StateMachine.add('setPosition',
                                   SetPositionState(recallWalls[borderName]),
                                   transitions={'succeeded':'BackwardOrder', 'timeout':'problem'})
            
            smach.StateMachine.add('BackwardOrder',
                                   OpenLoopOrder(-1,0.0,0.0, 
                                                 duration=6.00),
                                   transitions={'succeeded':'recaled', 'timeout':'problem'})
            
       
