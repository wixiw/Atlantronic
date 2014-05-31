#!/usr/bin/env python
# -*- coding: latin-1 -*-

import roslib; roslib.load_manifest('arp_master')
from arp_master.util.TableVierge import *

import rospy
import smach

from arp_master.util.Data import *
from arp_master.util.RobotVierge import *

#il faut voir ca comme un namespace
class Table2014(TableVierge):
    #
    # Points d'interet
    #     les points sont defini cote jaune comme toujours
    
    # La nomenclature des points est defini comme suit: P_SideTypePos
    # Side: S=Self / O=Opponent
    # Type: F=Fire / T=Torch / M=Mobile
    # Pos: T=Top / M=Mid / B=Bot

    
#Start Positions    
    P_START_POS         = Pose2D(1.430, 0.536 , 0.0)
    P_IN_FRONT_START_AREA   = Pose2D(1.100, 0.450, 0)

#Fire EntryPoint

    P_SELF_FIRE_TOP       =Pose2D(0.600, 0.400, -pi)
    P_SELF_FIRE_MID       =Pose2D(1.100, -0.100, -pi/2)
    P_SELF_FIRE_BOT       =Pose2D(0.600, -0.600, 0)
    P_OPP_FIRE_BOT        =Pose2D(-0.600, -0.600, 0)
    P_OPP_FIRE_MID        =Pose2D(-1.100, -0.100, pi/2)
    P_OPP_FIRE_TOP        =Pose2D(-0.600, 0.400, -pi)

#Static Torch EntryPoint

    P_SELF_TORCH_MID       =Pose2D(1.485, 0.200, 0)
    P_SELF_TORCH_BOT       =Pose2D(0.200, -0.985, pi/2)
    P_OPP_TORCH_BOT        =Pose2D(-0.200, -0.985, -pi/2)
    P_OPP_TORCH_MID        =Pose2D(-1.485, 0.200, 0)

#Mobile Torch Position

    P_SELF_MOBILE_TORCH       = Point(0.600,-0.100)
    P_OPP_MOBILE_TORCH        = Point(-0.600,-0.100)
    MOBILE_TORCH_RADIUS  = 0.080

#Heat Positions

    P_YELLOW_HEAT_BOT       = Pose2D(1.500, -1.000, -pi/4 )
    HEAT_BOT_RADIUS          = 0.250
    
# 
# Memoire des actions
#
    objectOneEmpty = False

    
    
    @staticmethod
    def printStratInfo():
        rospy.loginfo("*********************************************")
        rospy.loginfo("StratInfo :")
        
        rospy.loginfo("")
        rospy.loginfo("*********************************************")
        
        
