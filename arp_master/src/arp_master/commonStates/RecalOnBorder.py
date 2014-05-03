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
        
        with self:
            smach.StateMachine.add('AlignTurrets',
                                   OpenLoopOrder(0.1,0.0,0.0, 
                                                 duration=0.1),
                                   transitions={'succeeded':'ForwardOrder', 'timeout':'problem'})

            smach.StateMachine.add('ForwardOrder',
                                   ForwardOrder(dist=0.500,vmax=0.2),
                                   transitions={'succeeded':'non-recaled', 'timeout':'Pichenette1'}) 
            
            vpichenette=0.2
            postourelle=0.155
            #c'est une rotation autour de la roue droite et de la roue gauche
            smach.StateMachine.add('Pichenette1',
                                   OpenLoopOrder(vpichenette,0.0,vpichenette/postourelle, 
                                                 duration=0.4),
                                   transitions={'succeeded':'Pichenette2', 'timeout':'problem'})
            smach.StateMachine.add('Pichenette2',
                                   OpenLoopOrder(vpichenette,0.0,-vpichenette/postourelle, 
                                                 duration=0.4),
                                   transitions={'succeeded':'Pichenette3', 'timeout':'problem'})
            smach.StateMachine.add('Pichenette3',
                                   OpenLoopOrder(0.1,0.0,0.0, 
                                                 duration=0.1),
                                   transitions={'succeeded':'setPosition', 'timeout':'problem'})
            
            
            smach.StateMachine.add('setPosition',
                                   SetPositionState(recallWalls[borderName]),
                                   transitions={'succeeded':'BackwardOrder', 'timeout':'non-recaled'})
            
            smach.StateMachine.add('BackwardOrder',
                                   OpenLoopOrder(-0.2,0.0,0.0, 
                                                 duration=1.0),
                                   transitions={'succeeded':'recaled', 'timeout':'problem'})
            
       

